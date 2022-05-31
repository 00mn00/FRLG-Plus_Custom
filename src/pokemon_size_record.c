#include "global.h"
#include "gflib.h"
#include "data.h"
#include "event_data.h"
#include "pokedex.h"
#include "text.h"
#include "strings.h"
#include "constants/pokemon_size_record.h"

#define DEFAULT_MAX_SIZE 0 // was 0x8100 in Ruby/Sapphire, 0x8000 in Emerald

struct UnknownStruct
{
    u16 unk0;
    u8 unk2;
    u16 unk4;
};

static const struct UnknownStruct sBigMonSizeTable[] =
{
    {  290,   1,      0 },
    {  300,   1,     10 },
    {  400,   2,    110 },
    {  500,   4,    310 },
    {  600,  20,    710 },
    {  700,  50,   2710 },
    {  800, 100,   7710 },
    {  900, 150,  17710 },
    { 1000, 150,  32710 },
    { 1100, 100,  47710 },
    { 1200,  50,  57710 },
    { 1300,  20,  62710 },
    { 1400,   5,  64710 },
    { 1500,   2,  65210 },
    { 1600,   1,  65410 },
    { 1700,   1,  65510 },
};

static const u8 sGiftRibbonsMonDataIds[] =
{
    MON_DATA_MARINE_RIBBON, MON_DATA_LAND_RIBBON, MON_DATA_SKY_RIBBON,
    MON_DATA_COUNTRY_RIBBON, MON_DATA_NATIONAL_RIBBON, MON_DATA_EARTH_RIBBON,
    MON_DATA_WORLD_RIBBON
};

extern const u8 gText_DecimalPoint[];

static const u8 gText_Centimeter[] = _("centimeter");
static const u8 gText_CentimeterPlural[] = _("centimeters");
static const u8 gText_CentimeterSymbol[] = _("cm");
static const u8 gText_Meter[] = _("meter");
static const u8 gText_MeterPlural[] = _("meters");
static const u8 gText_MeterSymbol[] = _("m");
static const u8 gText_Kilogram[] = _("kilogram");
static const u8 gText_KilogramPlural[] = _("kilograms");
static const u8 gText_KilogramSymbol[] = _("kg");

static const u8* const sMetricText[] =
{
    [UNIT_TEXT_LENGTH_SMALL_SINGLE] = gText_Centimeter,
    [UNIT_TEXT_LENGTH_SMALL_PLURAL] = gText_CentimeterPlural,
    [UNIT_TEXT_LENGTH_SMALL_SYMBOL] = gText_CentimeterSymbol,
    [UNIT_TEXT_LENGTH_MEDIUM_SINGLE] = gText_Meter,
    [UNIT_TEXT_LENGTH_MEDIUM_PLURAL] = gText_MeterPlural,
    [UNIT_TEXT_LENGTH_MEDIUM_SYMBOL] = gText_MeterSymbol,
    [UNIT_TEXT_WEIGHT_SINGLE]        = gText_Kilogram,
    [UNIT_TEXT_WEIGHT_PLURAL]        = gText_KilogramPlural,
    [UNIT_TEXT_WEIGHT_SYMBOL]        = gText_KilogramSymbol
};


#define CM_PER_INCH 2.54

static u32 GetMonSizeHash(struct Pokemon * pkmn)
{
    u16 personality = GetMonData(pkmn, MON_DATA_PERSONALITY);
    u16 hpIV = GetMonData(pkmn, MON_DATA_HP_IV) & 0xF;
    u16 attackIV = GetMonData(pkmn, MON_DATA_ATK_IV) & 0xF;
    u16 defenseIV = GetMonData(pkmn, MON_DATA_DEF_IV) & 0xF;
    u16 speedIV = GetMonData(pkmn, MON_DATA_SPEED_IV) & 0xF;
    u16 spAtkIV = GetMonData(pkmn, MON_DATA_SPATK_IV) & 0xF;
    u16 spDefIV = GetMonData(pkmn, MON_DATA_SPDEF_IV) & 0xF;
    u32 hibyte = ((attackIV ^ defenseIV) * hpIV) ^ (personality & 0xFF);
    u32 lobyte = ((spAtkIV ^ spDefIV) * speedIV) ^ (personality >> 8);

    return (hibyte << 8) + lobyte;
}

static u8 TranslateBigMonSizeTableIndex(u16 a)
{
    u32 i;

    for (i = 1; i < 15; i++)
    {
        if (a < sBigMonSizeTable[i].unk4)
            return i - 1;
    }
    return i;
}

static u32 GetMonSize(u16 species, u16 b)
{
    u64 unk2;
    u64 unk4;
    u64 unk0;
    u32 height;
    u32 var;

    height = GetPokedexHeightWeight(SpeciesToNationalPokedexNum(species), 0);
    var = TranslateBigMonSizeTableIndex(b);
    unk0 = sBigMonSizeTable[var].unk0;
    unk2 = sBigMonSizeTable[var].unk2;
    unk4 = sBigMonSizeTable[var].unk4;
    unk0 += (b - unk4) / unk2;
    return height * unk0 / 10;
}

static void FormatMonSizeRecord(u8 *string, u32 size)
{
    string = ConvertIntToDecimalStringN(string, size / 1000, STR_CONV_MODE_LEFT_ALIGN, 8);
    string = StringAppend(string, gText_DecimalPoint);
    ConvertIntToDecimalStringN(string, size % 1000, STR_CONV_MODE_LEFT_ALIGN, 3);
}

static u8 CompareMonSize(u16 species, u16 *sizeRecord)
{
    if (gSpecialVar_Result >= PARTY_SIZE)
    {
        return 0;
    }
    else
    {
        struct Pokemon * pkmn = &gPlayerParty[gSpecialVar_Result];

        if (GetMonData(pkmn, MON_DATA_IS_EGG) == TRUE || GetMonData(pkmn, MON_DATA_SPECIES) != species)
        {
            return 1;
        }
        else
        {
            u32 oldSize;
            u32 newSize;
            u16 sizeParams;

            *(&sizeParams) = GetMonSizeHash(pkmn);
            newSize = GetMonSize(species, sizeParams);
            oldSize = GetMonSize(species, *sizeRecord);
            FormatMonSizeRecord(gStringVar3, oldSize);
            FormatMonSizeRecord(gStringVar2, newSize);
            if (newSize == oldSize)
            {
                return 4;
            }
            else if (newSize < oldSize)
            {
                return 2;
            }
            else
            {
                *sizeRecord = sizeParams;
                return 3;
            }
        }
    }
}

// Stores species name in gStringVar1, trainer's name in gStringVar2, and size in gStringVar3
static void GetMonSizeRecordInfo(u16 species, u16 *sizeRecord)
{
    u32 size = GetMonSize(species, *sizeRecord);

    FormatMonSizeRecord(gStringVar3, size);
    StringCopy(gStringVar1, gSpeciesNames[species]);
}

void BufferUnitSystemText(void)
{
    u8 *strvar, *text;
    u8 textType = gSpecialVar_0x8000;
    u8 stringVarNum = gSpecialVar_0x8001;

    switch (stringVarNum)
    {
        case 1:
        default:
            strvar = gStringVar1;
            break;
        case 2:
            strvar = gStringVar2;
            break;
        case 3:
            strvar = gStringVar3;
            break;
    }

    StringCopy(strvar, sMetricText[textType]);
}

void InitHeracrossSizeRecord(void)
{
    VarSet(VAR_HERACROSS_SIZE_RECORD, DEFAULT_MAX_SIZE);
}

void GetHeracrossSizeRecordInfo(void)
{
    u16 *sizeRecord = GetVarPointer(VAR_HERACROSS_SIZE_RECORD);

    GetMonSizeRecordInfo(SPECIES_HERACROSS, sizeRecord);
}

void CompareHeracrossSize(void)
{
    u16 *sizeRecord = GetVarPointer(VAR_HERACROSS_SIZE_RECORD);

    gSpecialVar_Result = CompareMonSize(SPECIES_HERACROSS, sizeRecord);
}

void InitMagikarpSizeRecord(void)
{
    VarSet(VAR_MAGIKARP_SIZE_RECORD, DEFAULT_MAX_SIZE);
}

void GetMagikarpSizeRecordInfo(void)
{
    u16 *sizeRecord = GetVarPointer(VAR_MAGIKARP_SIZE_RECORD);

    GetMonSizeRecordInfo(SPECIES_MAGIKARP, sizeRecord);
}

void CompareMagikarpSize(void)
{
    u16 *sizeRecord = GetVarPointer(VAR_MAGIKARP_SIZE_RECORD);

    gSpecialVar_Result = CompareMonSize(SPECIES_MAGIKARP, sizeRecord);
}

void GiveGiftRibbonToParty(u8 index, u8 ribbonId)
{
    s32 i;
    bool32 gotRibbon = FALSE;
    u8 data = 1;
    u8 array[8];
    memcpy(array, sGiftRibbonsMonDataIds, sizeof(sGiftRibbonsMonDataIds));

    if (index < 11 && ribbonId < 65)
    {
        gSaveBlock1Ptr->giftRibbons[index] = ribbonId;
        for (i = 0; i < PARTY_SIZE; i++)
        {
            struct Pokemon * mon = &gPlayerParty[i];

            if (GetMonData(mon, MON_DATA_SPECIES) != SPECIES_NONE && !GetMonData(mon, MON_DATA_SANITY_IS_EGG))
            {
                SetMonData(mon, array[index], &data);
                gotRibbon = TRUE;
            }
        }
        if (gotRibbon)
            FlagSet(FLAG_SYS_RIBBON_GET);
    }
}
