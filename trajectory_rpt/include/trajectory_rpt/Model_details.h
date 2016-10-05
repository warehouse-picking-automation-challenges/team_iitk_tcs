#ifndef MODEL_DETAILS_H__
#define MODEL_DETAILS_H__

int no_of_classes = 39;

std::string model_names[] = {

    "dove_beauty_bar"             , "rawlings_baseball",
    "clorox_utility_brush"        , "dr_browns_bottle_brush",
    "dasani_water_bottle"         , "easter_turtle_sippy_cup",
    "cherokee_easy_tee_shirt"     , "folgers_classic_roast_coffee",
    "crayola_24_ct"               , "peva_shower_curtain_liner",

    "barkely_hide_bones"          , "kyjen_squeakin_eggs_plush_puppies",
    "expo_dry_erase_board_eraser" , "scotch_duct_tape",
    "jane_eyre_dvd"               , "scotch_bubble_mailer",
    "woods_extension_cord"        , "womens_knit_gloves",
    "cool_shot_glue_sticks"       , "elmers_washable_no_run_school_glue",

    "staples_index_cards"         , "laugh_out_loud_joke_book",
    "i_am_a_bunny_book"           , "kleenex_tissue_box",
    "soft_white_lightbulb"        , "kleenex_paper_towels"	,
    "rolodex_jumbo_pencil_cup"    , "ticonderoga_12_pencils",
    "platinum_pets_dog_bowl"      , "hanes_tube_socks",

    "creativity_chenille_stems"   , "fiskars_scissors_red",
    "cloud_b_plush_bear"          , "safety_first_outlet_plugs",
    "fitness_gear_3lb_dumbbell"   , "oral_b_toothbrush_green",
    "up_glucose_bottle"           , "command_hooks",
    "oral_b_toothbrush_red"
};

int TOTAL_NO_OF_BACKPROJECTIONS = 5;

bool LOOKUP_POSTERIORS[]={
    1,1,0,1,1     , 1,1,0,1,1,
    1,1,1,0,1     , 1,1,0,1,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,1,0,1     , 1,1,0,1,1,

    1,1,1,0,1     , 1,1,1,0,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,0,1,1     , 1,1,1,0,1,
    1,1,0,0,1     , 1,1,0,1,1,
    1,1,0,1,1     , 1,1,0,1,1,

    1,1,1,0,1     , 1,1,1,0,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,0,1,1     , 1,1,0,1,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,0,1,1     , 1,1,0,1,1,

    1,1,1,0,1     , 1,1,1,0,1,
    1,1,0,0,1     , 1,1,1,0,1,
    1,1,1,0,1     , 1,1,1,0,1,
    1,1,0,1,1     , 1,1,0,1,1,
    1,1,1,0,1
};

const int CONST_MEAN_VAR =0;
const int CONST_PRINCIPLE_CURVATURE = 1;
const int CONST_TRUE_COLOR = 2;
const int CONST_GRAY_SHADES = 3;
const int CONST_RANDOM_FOREST = 4;


#endif









