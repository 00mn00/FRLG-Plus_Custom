Gym_EventScript_MissingBadgeTrigger_BoulderBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_BoulderBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_CascadeBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_CascadeBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_ThunderBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_ThunderBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_RainbowBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_RainbowBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_SoulBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_SoulBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_MarshBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_MarshBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_VolcanoBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_VolcanoBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

Gym_EventScript_MissingBadgeTrigger_EarthBadge::
	lockall
	msgbox Text_MissingBadgeTrigger_EarthBadge, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveUp
	waitmovement 0
	releaseall
	end

EventScript_MoveUp::
	walk_up
	step_end

RocketHideout_EventScript_PreGiovanniTrigger::
	lockall
	msgbox Text_RocketHideoutTrappedTrigger, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveRight
	waitmovement 0
	releaseall
	end

EventScript_MoveRight::
	walk_right
	step_end

Gym_EventScript_MissingNecessaryHMTrigger::
	lockall
	msgbox Text_MissingNecessaryHMTrigger, MSGBOX_SIGN
	applymovement OBJ_EVENT_ID_PLAYER, EventScript_MoveDown
	waitmovement 0
	releaseall
	end

EventScript_MoveDown::
	walk_down
	step_end

Text_MissingBadgeTrigger_BoulderBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat BROCK\n"
	.string "and win his BOULDER BADGE!$"

Text_MissingBadgeTrigger_CascadeBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat MISTY\n"
	.string "and win her CASCADE BADGE!$"

Text_MissingBadgeTrigger_ThunderBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat LT.\n"
	.string "SURGE and win his THUNDER BADGE!$"

Text_MissingBadgeTrigger_RainbowBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat ERIKA\n"
	.string "and win her RAINBOW BADGE!$"

Text_MissingBadgeTrigger_SoulBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat KOGA\n"
	.string "and win his SOUL BADGE!$"

Text_MissingBadgeTrigger_MarshBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat SABRINA\n"
	.string "and win her MARSH BADGE!$"

Text_MissingBadgeTrigger_VolcanoBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat BLAINE\n"
	.string "and win his VOLCANO BADGE!$"

Text_MissingBadgeTrigger_EarthBadge::
	.string "I want to be the very best…\p"
	.string "I won't give up until I beat\n"
	.string "GIOVANNI, win his EARTH BADGE,\l"
	.string "and bring him to justice!$"

Text_RocketHideoutTrappedTrigger::
	.string "I want to be the very best…\p"
	.string "I won't leave until I've brought the\n"
	.string "notorious TEAM ROCKET to justice!$"

Text_MissingNecessaryHMTrigger::
	.string "I want to be the very best…\p"
	.string "I don't have a POKéMON with me\n"
	.string "that can learn the HM needed to\l"
	.string "challenge this GYM!$"
