.set LOCALID_GROWLITHE_MASTER, 2
.set TRAINER, VAR_0x8009

Route7_MapScripts:: @ 8167FBB
	map_script MAP_SCRIPT_ON_TRANSITION, Route7_MapScript_SetMasterTrainerMovementTypes
	.byte 0

Route7_MapScript_SetMasterTrainerMovementTypes::
	setvar TRAINER, MASTER_GROWLITHE
	callnative ShouldChangeMasterTrainerMovementType
	compare VAR_RESULT, FALSE
	call_if_eq Route7_MapScript_GrowlitheMasterMovementType
	end

Route7_MapScript_GrowlitheMasterMovementType::
	setobjectmovementtype LOCALID_GROWLITHE_MASTER, MOVEMENT_TYPE_VS_SEEKER_4D
	return

Route7_EventScript_UndergroundPathSign:: @ 8167FBC
	msgbox Route7_Text_UndergroundPathSign, MSGBOX_SIGN
	end

Route111_EventScript_Girl::
	lock
	faceplayer
	dotimebasedevents
	goto_if_set FLAG_DAILY_ROUTE_111_RECEIVED_BERRY, Route111_EventScript_ReceivedBerry
	msgbox Route111_Text_WateredPlantsEveryDayTakeBerry, MSGBOX_DEFAULT
	giveitem ITEM_RAZZ_BERRY
	compare VAR_RESULT, FALSE
	goto_if_eq EventScript_BagIsFull
	setflag FLAG_DAILY_ROUTE_111_RECEIVED_BERRY
	msgbox Route111_Text_GoingToTryToMakeDifferentColorBerries, MSGBOX_DEFAULT
	release
	end

Route111_EventScript_ReceivedBerry::
	msgbox Route111_Text_WhatColorBerriesToLookForToday, MSGBOX_DEFAULT
	release
	end