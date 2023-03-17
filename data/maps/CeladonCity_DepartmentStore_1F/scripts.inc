CeladonCity_DepartmentStore_1F_MapScripts:: @ 816BAD8
	.byte 0

CeladonCity_DepartmentStore_1F_EventScript_Receptionist:: @ 816BAD9
	goto_if_questlog EventScript_ReleaseEnd
	lock
	faceplayer
	message CeladonCity_DepartmentStore_1F_Text_WelcomeToDeptStore
	waitmessage
	pokemart CeladonCity_DepartmentStore_1F_Mail
	msgbox CeladonCity_DepartmentStore_1F_Text_PleaseComeAgain
	release
	end

	.align 2
CeladonCity_DepartmentStore_1F_Mail::
	.2byte ITEM_BEAD_MAIL
	.2byte ITEM_DREAM_MAIL
	.2byte ITEM_FAB_MAIL
	.2byte ITEM_GLITTER_MAIL
	.2byte ITEM_HARBOR_MAIL
	.2byte ITEM_MECH_MAIL
	.2byte ITEM_ORANGE_MAIL
	.2byte ITEM_RETRO_MAIL
	.2byte ITEM_SHADOW_MAIL
	.2byte ITEM_TROPIC_MAIL
	.2byte ITEM_WAVE_MAIL
	.2byte ITEM_WOOD_MAIL
	.2byte POKEMART_LIST_END
	release
	end

CeladonCity_DepartmentStore_1F_EventScript_LayoutSign:: @ 816BAE2
	msgbox CeladonCity_DepartmentStore_1F_Text_FloorDescriptions, MSGBOX_SIGN
	end

CeladonCity_DepartmentStore_1F_EventScript_FloorSign:: @ 816BAEB
	msgbox CeladonCity_DepartmentStore_1F_Text_ServiceCounter, MSGBOX_SIGN
	end
