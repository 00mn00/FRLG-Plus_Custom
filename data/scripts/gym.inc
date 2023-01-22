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

Text_GymNotDoneYet:: @ 81BFB6E
	.string "I'm not done here yet!$"