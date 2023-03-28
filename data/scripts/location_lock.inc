Gym_EventScript_MissingBadgeTrigger::
	lockall
	msgbox Text_GymNotDoneYet, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, Gym_EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MoveUp::
	walk_up
	step_end

RocketHideout_EventScript_PreGiovanniTrigger::
	lockall
	msgbox Text_RocketHideoutTrapped, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, RocketHideout_EventScript_MoveRight
	waitmovement 0
	releaseall
	end

RocketHideout_EventScript_MoveRight::
	walk_right
	step_end

Text_GymNotDoneYet::
	.string "I want to be the very best…\n"
	.string "I won't give up until I've defeated\l"
	.string "this gym leader and won their badge!$"

Text_RocketHideoutTrapped::
	.string "I won't leave until I've brought the\n"
	.string "notorious TEAM ROCKET to justice!$"