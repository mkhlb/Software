#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/logger/logger.h"

struct PivotKickFSM
{
    struct TurnToKickOriginFSM
    {
        struct ControlParams
        {

        };

        DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

        void turnToKickOrigin(
                const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent);


    };

    struct PossessAndDribbleFSM
    {
        class StartState;

        struct ControlParams
        {
            // The location where the kick will be taken from
            Point kick_origin;
        };

        DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

        /**
         * Action that updates the DribbleFSM to get possession of the ball and drive it to the shot spot
         *
         * @param event PivotKickFSM::Update event
         * @param processEvent processes the GetBehindBallFSM::Update
         */
        void getPossessionAndDrive(
                const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent);

        auto operator()()
        {
            using namespace boost::sml;

            DEFINE_SML_STATE(StartState)
            DEFINE_SML_STATE(DribbleFSM)
            DEFINE_SML_EVENT(Update)

            DEFINE_SML_SUB_FSM_UPDATE_ACTION(getPossessionAndDrive, DribbleFSM)

            return make_transition_table(
                    // src_state + event [guard] / action = dest_state
                    *StartState_S + Update_E / getPossessionAndDrive_A = DribbleFSM_S,
                    DribbleFSM_S + Update_E / getPossessionAndDrive_A, DribbleFSM_S = X,
                    X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
        }
    };

    class PossessBall;
    class KickState;

    struct ControlParams
    {
        // The location where the kick will be taken from
        Point kick_origin;
        // The direction the Robot will kick in
        Angle kick_direction;
        // How the robot will chip or kick the ball
        AutoChipOrKick auto_chip_or_kick;
    };

    // this struct defines the only event that the PivotKickFSM responds to
    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    void possess(const Update& event);

    void turnToKick(const Update& event, boost::sml::back::process<>)

    /**
     * Action that kicks the ball
     *
     * @param event PivotKickFSM::Update event
     */
    void kickBall(const Update& event);

    /**
     * Action that updates the DribbleFSM to pivot to shoot the ball at the target
     * @param event
     */
    void pivot(const Update& event, boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * Guard that checks if the ball has been kicked
     *
     * @param event PivotKickFSM::Update event
     *
     * @return if the ball has been kicked
     */
    bool ballKicked(const Update& event);




    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(PossessAndDribbleFSM)
        DEFINE_SML_STATE(KickState)
        DEFINE_SML_STATE(DribbleFSM)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(ballKicked)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(possessAndDribble, PossessAndDribbleFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(pivot, DribbleFSM)
        DEFINE_SML_ACTION(kickBall)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *PossessAndDribbleFSM_S + Update_E / possessAndDribble_A,
            PossessAndDribbleFSM_S = DribbleFSM_S,
            DribbleFSM_S + Update_E / pivot_A, DribbleFSM_S = KickState_S,
            KickState_S + Update_E[!ballKicked_G] / kickBall_A,
            KickState_S + Update_E[ballKicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION                         = X);
    }
};
