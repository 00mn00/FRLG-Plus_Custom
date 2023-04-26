PalletTown_PlayersHouse_2F_MapScripts:: @ 8168CA3
	map_script MAP_SCRIPT_ON_TRANSITION, PalletTown_PlayersHouse_2F_OnTransition
	map_script MAP_SCRIPT_ON_WARP_INTO_MAP_TABLE, PalletTown_PlayersHouse_2F_OnWarp
	.byte 0

PalletTown_PlayersHouse_2F_OnTransition:: @ 8168CAE
	compare VAR_MAP_SCENE_PALLET_TOWN_PLAYERS_HOUSE_2F, 0
	call_if_eq PalletTown_PlayersHouse_2F_EventScript_SetRespawn
	end

PalletTown_PlayersHouse_2F_EventScript_SetRespawn:: @ 8168CBA
	setrespawn SPAWN_PALLET_TOWN
	return

PalletTown_PlayersHouse_2F_OnWarp:: @ 8168CBE
	map_script_2 VAR_MAP_SCENE_PALLET_TOWN_PLAYERS_HOUSE_2F, 0, PalletTown_PlayersHouse_2F_FirstWarpIn
	.2byte 0

PalletTown_PlayersHouse_2F_FirstWarpIn:: @ 8168CC8
	turnobject OBJ_EVENT_ID_PLAYER, DIR_NORTH
	setvar VAR_MAP_SCENE_PALLET_TOWN_PLAYERS_HOUSE_2F, 1
	end

PalletTown_PlayersHouse_2F_EventScript_NES:: @ 8168CD2
	msgbox PalletTown_PlayersHouse_2F_Text_PlayedWithNES, MSGBOX_SIGN
	end

PalletTown_PlayersHouse_2F_EventScript_Sign:: @ 8168CDB
	msgbox PalletTown_PlayersHouse_2F_Text_PressLRForHelp, MSGBOX_SIGN
	end

PalletTown_PlayersHouse_2F_EventScript_WallClock::
	lockall
	checkplayergender
	copyvar 0x8004, VAR_RESULT
	goto_if_set FLAG_SET_WALL_CLOCK, PlayersHouse_2F_EventScript_CheckWallClock
	msgbox PlayersHouse_2F_Text_ClockIsStopped, MSGBOX_DEFAULT
	call PlayersHouse_2F_EventScript_SetWallClock
	delay 30
	setflag FLAG_SET_WALL_CLOCK
	setvar VAR_PLAYERS_HOUSE_CLOCK_SET, 1
	releaseall
	end

PlayersHouse_2F_EventScript_CheckWallClock:: @ 829283F
	incrementgamestat GAME_STAT_CHECKED_CLOCK
	fadescreen FADE_TO_BLACK
	special Special_ViewWallClock
	waitstate
	releaseall
	end

PlayersHouse_2F_EventScript_SetWallClock:: @ 8292849
	fadescreen FADE_TO_BLACK
	special StartWallClock
	waitstate
	return

PalletTown_PlayersHouse_2F_EventScript_PC:: @ 8168CE4
	goto_if_questlog EventScript_ReleaseEnd
	lockall
	setvar VAR_0x8004, HELPCONTEXT_BEDROOM_PC
	special Script_SetHelpContext
	setvar VAR_0x8004, 1
	special AnimatePcTurnOn
	playse SE_PC_ON
	msgbox Text_PlayerBootedUpPC
	special BedroomPC
	waitstate
	special SetHelpContextForMap
	releaseall
	end

EventScript_PalletTown_PlayersHouse_2F_ShutDownPC:: @ 8168D17
	setvar VAR_0x8004, 1
	playse SE_PC_OFF
	special AnimatePcTurnOff
	special SetHelpContextForMap
	releaseall
	end

EventScript_PalletTown_PlayersHouse_2F_ClockSetTriggerBottom::
	lockall
	setvar VAR_TEMP_6, 1
	goto EventScript_PalletTown_PlayersHouse_2F_ClockSetTrigger
	end

EventScript_PalletTown_PlayersHouse_2F_ClockSetTriggerRight::
	lockall
	setvar VAR_TEMP_6, 2
	goto EventScript_PalletTown_PlayersHouse_2F_ClockSetTrigger
	end

EventScript_PalletTown_PlayersHouse_2F_ClockSetTrigger::
	applymovement OBJ_EVENT_ID_PLAYER, Movement_WalkInPlaceFastestLeft
	waitmovement 0
	playse SE_PIN
	applymovement OBJ_EVENT_ID_PLAYER, Movement_ExclamationMark
	waitmovement 0
	delay 30
	msgbox PlayersHouse_2F_Text_ClockIsStopped
	closemessage
	compare VAR_TEMP_6, 1
	call_if_eq EventScript_PalletTown_PlayersHouse_2F_ClockSetFromSouthOfStairs
	compare VAR_TEMP_6, 2
	call_if_eq EventScript_PalletTown_PlayersHouse_2F_ClockSetFromEastOfStairs
	checkplayergender
	copyvar 0x8004, VAR_RESULT
	call PlayersHouse_2F_EventScript_SetWallClock
	delay 30
	setflag FLAG_SET_WALL_CLOCK
	setvar VAR_PLAYERS_HOUSE_CLOCK_SET, 1
	releaseall
	end

EventScript_PalletTown_PlayersHouse_2F_ClockSetFromSouthOfStairs:: @ 81F0F7C
	applymovement OBJ_EVENT_ID_PLAYER, Movement_PalletTown_PlayersHouse_2F_ClockSetFromSouthOfStairs
	waitmovement 0
	return

EventScript_PalletTown_PlayersHouse_2F_ClockSetFromEastOfStairs:: @ 81F0F87
	applymovement OBJ_EVENT_ID_PLAYER, Movement_PalletTown_PlayersHouse_2F_ClockSetFromEastOfStairs
	waitmovement 0
	return

Movement_PalletTown_PlayersHouse_2F_ClockSetFromSouthOfStairs::
	walk_down
	walk_left
	walk_left
	walk_left
	walk_up
	walk_left
	walk_up
	step_end

Movement_PalletTown_PlayersHouse_2F_ClockSetFromEastOfStairs::
	walk_down
	walk_down
	walk_left
	walk_left
	walk_left
	walk_left
	walk_up
	walk_left
	walk_up
	step_end