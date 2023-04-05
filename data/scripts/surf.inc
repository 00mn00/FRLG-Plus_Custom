EventScript_UseSurf:: @ 81A6AC8
	goto_if_questlog EventScript_ReleaseEnd
	lockall
	goto_if_unset FLAG_BADGE05_GET, EventScript_CantSurf
	checkpartymove MOVE_SURF
	compare VAR_RESULT, PARTY_SIZE
	goto_if_eq EventScript_CantSurf
	setfieldeffectarg 0, VAR_RESULT
	getpartymonname 0, VAR_RESULT
	getmovename 1, MOVE_SURF
	msgbox Text_WantToSurf, MSGBOX_YESNO
	compare VAR_RESULT, NO
	goto_if_eq EventScript_EndSurf
	msgbox Text_UsedSurf
	dofieldeffect FLDEFF_USE_SURF
	goto EventScript_EndSurf
	end

EventScript_EndSurf:@ 81A6B0B
	closemessage
	releaseall
	end

EventScript_CantSurf: @ 81A6B0C
	lockall
	msgbox Text_SurfableWater, MSGBOX_SIGN
	releaseall
	end

EventScript_CurrentTooFast:: @ 81A6B0D
	lockall
	msgbox Text_CurrentTooFast, MSGBOX_SIGN
	releaseall
	end
