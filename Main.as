// ============================================================
//  FPV Drone Camera — plugin Openplanet pour Trackmania 2020
//  Physique Acro — orientation en espace CORPS (body-frame)
//
//  Mode 2 — Stick gauche : Throttle (Y) + Yaw (X)
//            Stick droit  : Pitch (Y) + Roll (X)
//
//  Architecture :
//    Main()         → physique (orientation, vélocité, position)
//    RenderUpdate() → application des angles TM, juste avant le
//                     rendu, APRÈS que TM a traité ses propres
//                     inputs joystick. Élimine le conflit stick droit.
//
//  Nécessite la Cam 7 (touche 7) dans l'éditeur de replay.
// ============================================================

// ── RÉGLAGES (persistés dans physics.txt, UI dans RenderSettings) ──

bool  S_Enabled        = true;
float S_Thrust         = 246.0f;
float S_TiltRate       = 10.0f;
float S_YawRate        = 10.0f;
float S_AngInertia     = 0.08f;
float S_CameraTilt     = -54.0f;
bool  S_VisualRoll     = true;
float S_Drag           = 0.024f;
float S_DragY          = 0.032f;
float S_QuadDrag       = 0.0f;
float S_QuadDragY      = 0.0f;
bool  S_GravityEnabled = true;
float S_Gravity        = 50.0f;
float S_Deadzone       = 0.0f;
bool  S_UseMinHeight   = true;
float S_MinHeight      = 9.0f;
float S_ThrottleMax    = 1.0f;
float S_ThrottleCurve  = 1.0f;
float S_MinThrottlePct = 0.0f;
float S_MotorTau       = 0.02f;
float S_GyroCoeff      = 1.0f;

// ── OVERLAY INPUTS ───────────────────────────────────────────
int S_OverlayX = 16;
int S_OverlayY = 720;

// Esthétique overlay (persistés dans misc.txt, configurables dans l'onglet Misc)
bool  S_ShowInputOverlay = true;
bool  g_ShowLiveValues   = false;
float g_OvSz          = 72.0f;
float g_OvGap         = 10.0f;
float g_OvBgAlpha     = 0.72f;
float g_OvBorderAlpha = 0.45f;
float g_OvCrossAlpha  = 0.22f;
float g_OvDotSize     = 4.5f;
float g_OvDotR        = 1.0f;
float g_OvDotG        = 1.0f;
float g_OvDotB        = 1.0f;

// ── BINDS (persistés dans binds.txt, pas via [Setting]) ──────
int S_RecPadBtn     = 6;  // R1
int S_RecKey        = 0;  // None
int S_RespawnPadBtn = 2;  // B
int S_RespawnKey    = 0;  // None
int S_CpPadBtn      = 5;  // L1
int S_CpKey         = 0;  // None
int S_RewindPadBtn  = 3;  // X
int S_RewindKey     = 0;  // None

// ── INVERSIONS D'AXES ────────────────────────────────────────
bool S_InvThrottle   = false;
bool S_InvYaw        = false;
bool S_InvPitch      = false;
bool S_InvRoll       = false;
bool S_InvVisualRoll = false;

// ── TRAJECTOIRE ───────────────────────────────────────────────
bool S_Rec       = false;
bool S_Play      = false;
int S_KfFreq   = 20;  // Hz (keyframes per second)

// ── ÉTAT PHYSIQUE ─────────────────────────────────────────────

vec3  g_Fwd            = vec3(0, 0, 1);
vec3  g_Up             = vec3(0, 1, 0);
float g_PitchRate      = 0.0f;
float g_RollRate       = 0.0f;
float g_YawRateV       = 0.0f;
vec3  g_Vel            = vec3(0, 0, 0);
vec3  g_Pos            = vec3(0, 0, 0);
float g_Throttle       = 0.0f;
float g_MotorThrottle  = 0.0f;  // throttle filtré par motor lag
bool  g_WasOn          = false;
bool  g_PosInit        = false;

// Position/orientation de départ
int   g_CapturingBind = -1; // -1=rien, 0=RecPad,1=RecKey,2=RespawnPad,3=RespawnKey,4=CpPad,5=CpKey,6=RewindPad,7=RewindKey
vec3  g_StartPos = vec3(0, 0, 0);
vec3  g_StartFwd = vec3(0, 0, 1);
vec3  g_StartUp  = vec3(0, 1, 0);
bool  g_RecWasDown        = false;
bool  g_RespawnWasDown    = false;
bool  g_CpWasDown         = false;
bool  g_RewindWasDown     = false;
bool  g_Rewinding         = false;
bool  g_RecBeforeRewind   = false;

// Checkpoint virtuel
bool  g_HasCheckpoint  = false;
vec3  g_CpPos          = vec3(0,0,0);
vec3  g_CpFwd          = vec3(0,0,1);
vec3  g_CpUp           = vec3(0,1,0);
vec3  g_CpVel          = vec3(0,0,0);
float g_CpPitchRate    = 0.0f;
float g_CpRollRate     = 0.0f;
float g_CpYawRate      = 0.0f;
float g_CpThrottle     = 0.0f;
float g_CpMotorThr     = 0.0f;
float g_CpRecTime      = 0.0f;
uint  g_CpPadPressMs   = 0;
bool  g_CpKeyWasDown   = false;
uint  g_CpKeyPressMs   = 0;

// Labels pour l'UI de bind
const array<string> PAD_LABELS = {
    "None","A","B","X","Y","L1","R1","L2","R2","L3","R3",
    "↑","↓","←","→","Menu","View"
};
const array<string> KEY_LABELS = {
    "None","F1","F2","F3","F4","F5","F6","F7","F8","F9","F10","F11","F12"
};

// Presets utilisateur (3 slots, fichiers dans PluginStorage/FPVDrone/)
const int NUM_PRESETS = 3;
array<string> g_PresetNames = {"Preset 1", "Preset 2", "Preset 3"};
bool g_PresetsLoaded = false;
string g_PresetStatus = "";
bool g_PresetError = false;

// Angles calculés par la physique, appliqués à la caméra dans RenderUpdate.
float g_CamYaw   = 0.0f;
float g_CamPitch = 0.0f;
float g_CamRoll  = 0.0f;

// ── DONNÉES DE TRAJECTOIRE ────────────────────────────────────

array<float> g_TrkT;
array<vec3>  g_TrkPos;
array<float> g_TrkYaw;
array<float> g_TrkPitch;
array<float> g_TrkRoll;
array<vec3>  g_TrkFwd;
array<vec3>  g_TrkUp;
array<vec3>  g_TrkVel;
float g_RecTime  = 0.0f;
float g_NextKfTime = 0.0f;
float g_PlayTime = 0.0f;
uint  g_PlayIdx  = 0;
bool   g_WasRec    = false;
bool   g_WasPlay   = false;
bool   g_Injecting = false;
string g_InjectStatus = "";
bool   g_InjectError  = false;

uint g_LastPhysUpdateMs = 0;
float g_FilteredDt = 1.0f / 120.0f;
string g_UncrashedPresetStatus = "";
string g_UncrashedTestReport = "";

// Offsets pour neutraliser l'input TM sur la caméra libre (propriétés read-only via API)
uint16 g_Off_RotSpd = 0;
uint16 g_Off_RotIn  = 0;

const float DEG2RAD = Math::PI / 180.0f;
const float RAD2DEG = 180.0f / Math::PI;

// ── LOCALISATION ─────────────────────────────────────────────
// 0=Français  1=English  2=Deutsch  3=Español  4=Русский  5=Polski
int g_Lang = 0;
const array<string> LANG_NAMES = {"Français","English","Deutsch","Español","Русский","Polski"};
array<string> g_Str;

const int LS_OVERLAY_SECT  =  0;
const int LS_SHOW_OVERLAY  =  1;
const int LS_SHOW_VALUES   =  2;
const int LS_POS_X         =  3;
const int LS_POS_Y         =  4;
const int LS_SZ            =  5;
const int LS_GAP           =  6;
const int LS_BG_ALPHA      =  7;
const int LS_BORDER_ALPHA  =  8;
const int LS_CROSS_ALPHA   =  9;
const int LS_DOT_SIZE      = 10;
const int LS_DOT_COLOR     = 11;
const int LS_PHYSICS_SECT  = 12;
const int LS_PRESET_SECT   = 13;
const int LS_UNCRASHED_BTN = 14;
const int LS_BIND_HINT1    = 15;
const int LS_BIND_HINT2    = 16;
const int LS_RECORD        = 17;
const int LS_RESPAWN       = 18;
const int LS_CHECKPOINT    = 19;
const int LS_PAD_LBL       = 20;
const int LS_KEY_LBL       = 21;
const int LS_PRESS_BTN     = 22;
const int LS_CP_HINT       = 23;
const int LS_DRONE_ON      = 24;
const int LS_CAM7          = 25;
const int LS_SPEED_LBL     = 26;
const int LS_THR_LBL       = 27;
const int LS_GRAV_LBL      = 28;
const int LS_ON_LBL        = 29;
const int LS_OFF_LBL       = 30;
const int LS_PRESETS_LBL   = 31;
const int LS_SAVE_LBL      = 32;
const int LS_LOAD_LBL      = 33;
const int LS_LANG_LBL      = 34;
const int LS_HELP_TAB      = 35;
const int LS_MISC_TAB      = 36;
const int LS_BINDS_TAB     = 37;
const int LS_HELP_WHAT     = 38;
const int LS_HELP_REQ      = 39;
const int LS_HELP_CTRL     = 40;
const int LS_HELP_PRESETS  = 42;
const int LS_HELP_REC     = 43;  // nouveau
const int LS_HELP_CP      = 44;  // nouveau
const int LS_STATUS_ON     = 45;
const int LS_STATUS_OFF    = 46;
// ── Physique tab ─────────────────────────────────────────────
const int LS_PHYSICS_TAB   = 47;
const int LS_ENABLED       = 48;
const int LS_THRUST        = 49;
const int LS_TILT_RATE     = 50;
const int LS_YAW_RATE      = 51;
const int LS_ANG_INERTIA   = 52;
const int LS_CAM_TILT      = 53;
const int LS_VISUAL_ROLL   = 54;
const int LS_DRAG_H        = 55;
const int LS_DRAG_V        = 56;
const int LS_QDRAG_H       = 57;
const int LS_QDRAG_V       = 58;
const int LS_GRAVITY_EN    = 59;
const int LS_GRAVITY       = 60;
const int LS_DEADZONE      = 61;
const int LS_MIN_H_EN      = 62;
const int LS_MIN_H         = 63;
const int LS_THR_MAX       = 64;
const int LS_THR_CURVE     = 65;
const int LS_MIN_THR_PCT   = 66;
const int LS_MOTOR_TAU     = 67;
const int LS_GYRO          = 68;
const int LS_DECIM_STEP    = 69;
const int LS_KF_FREQ      = 70;
// ── Axes tab ──────────────────────────────────────────────────
const int LS_AXES_TAB      = 71;
const int LS_INV_THROTTLE  = 71;
const int LS_INV_YAW       = 72;
const int LS_INV_PITCH     = 73;
const int LS_INV_ROLL      = 74;
const int LS_INV_VIS_ROLL  = 75;
const int LS_REWIND        = 76;
const int LS_EXPORT_SECT   = 77;
const int LS_EXPORT_STEP1  = 78;
const int LS_EXPORT_STEP2  = 79;
const int LS_EXPORT_STEP3  = 80;
const int LS_EXPORT_STEP4  = 81;
const int LS_EXPORT_BTN    = 82;
const int LS_EXPORT_LINK   = 83;
const int LS_LOC_COUNT     = 84;

string T(int id) {
    if (g_Str.Length == 0) InitLoc();
    if (id < 0 || id >= int(g_Str.Length)) return "?";
    return g_Str[id];
}

void InitLoc() {
    g_Str.Resize(LS_LOC_COUNT);

    // ── Français (défaut) ────────────────────────────────────
    g_Str[LS_OVERLAY_SECT]  = "\\$888── Overlay sticks ─────────────────────────────";
    g_Str[LS_SHOW_OVERLAY]  = "Afficher overlay sticks";
    g_Str[LS_SHOW_VALUES]   = "Afficher valeurs (pitch / roll / yaw)";
    g_Str[LS_POS_X]         = "Position X";
    g_Str[LS_POS_Y]         = "Position Y";
    g_Str[LS_SZ]            = "Taille cases";
    g_Str[LS_GAP]           = "Espacement";
    g_Str[LS_BG_ALPHA]      = "Fond alpha";
    g_Str[LS_BORDER_ALPHA]  = "Bordure alpha";
    g_Str[LS_CROSS_ALPHA]   = "Croix alpha";
    g_Str[LS_DOT_SIZE]      = "Taille point";
    g_Str[LS_DOT_COLOR]     = "Couleur point";
    g_Str[LS_PHYSICS_SECT]  = "\\$888── Physique actuelle ──────────────────────────";
    g_Str[LS_PRESET_SECT]   = "\\$888── Preset physique (inspiration Uncrashed) ────";
    g_Str[LS_UNCRASHED_BTN] = "Copy values from Uncrashed (5\" Freestyle)";
    g_Str[LS_BIND_HINT1]    = "\\$888Clique sur une case puis appuie sur le bouton/touche voulu.";
    g_Str[LS_BIND_HINT2]    = "Pad ET clavier peuvent être bindés en même temps. [×] = effacer.";
    g_Str[LS_RECORD]        = "Record";
    g_Str[LS_RESPAWN]       = "Respawn";
    g_Str[LS_CHECKPOINT]    = "Checkpoint";
    g_Str[LS_PAD_LBL]       = "Pad :";
    g_Str[LS_KEY_LBL]       = "Clavier :";
    g_Str[LS_PRESS_BTN]     = "< Appuie... >";
    g_Str[LS_CP_HINT]       = "\\$888Checkpoint : 1er appui = sauver position, 2e appui = y retourner. Long = effacer.";
    g_Str[LS_DRONE_ON]      = "FPV Drone actif";
    g_Str[LS_CAM7]          = "\\$888(Cam 7 requise)";
    g_Str[LS_SPEED_LBL]     = "Vitesse : ";
    g_Str[LS_THR_LBL]       = "Throttle : ";
    g_Str[LS_GRAV_LBL]      = "Grav : ";
    g_Str[LS_ON_LBL]        = "ON";
    g_Str[LS_OFF_LBL]       = "OFF";
    g_Str[LS_PRESETS_LBL]   = "\\$888Presets :";
    g_Str[LS_SAVE_LBL]      = "Sauv.";
    g_Str[LS_LOAD_LBL]      = "Charger";
    g_Str[LS_LANG_LBL]      = "Langue :";
    g_Str[LS_HELP_TAB]      = "Aide";
    g_Str[LS_MISC_TAB]      = "Misc";
    g_Str[LS_BINDS_TAB]     = "Binds";
    g_Str[LS_HELP_WHAT]     = "FPV Drone est un plugin caméra qui simule la physique d'un drone FPV en mode acrobatique. Il fonctionne dès que la Caméra 7 (caméra libre) est active.";
    g_Str[LS_HELP_REQ]      = "Une manette est requise.";
    g_Str[LS_HELP_CTRL]     = "Commandes : Stick gauche Y = Gaz (haut = plein gaz) - Stick gauche X = Lacet - Stick droit Y = Tangage - Stick droit X = Roulis.";
    g_Str[LS_HELP_REC]      = "Enregistrement : un appui court sur le bind Record démarre et arrête l'enregistrement. Pendant l'enregistrement, maintenir le bind Rewind fait reculer le drone dans le temps. Tu peux garder tes inputs actifs pendant le rewind pour reprendre la trajectoire fluidement au moment du relâchement.";
    g_Str[LS_HELP_CP]       = "Checkpoint : le premier appui sauvegarde la position courante, le prochain appui retourne à cette position. Un appui long efface le checkpoint.";
    g_Str[LS_HELP_PRESETS]  = "Presets : sauvegarde et charge tes réglages physiques via les 3 slots dans le HUD (bas gauche). Utile pour alterner entre plusieurs sensations de vol.";
    g_Str[LS_STATUS_ON]     = "actif";
    g_Str[LS_STATUS_OFF]    = "inactif";
    g_Str[LS_PHYSICS_TAB]   = "Physique";
    g_Str[LS_ENABLED]       = "Activé";
    g_Str[LS_THRUST]        = "Puissance moteurs / Thrust (u/s²)";
    g_Str[LS_TILT_RATE]     = "Vitesse inclinaison Pitch/Roll (rad/s)";
    g_Str[LS_YAW_RATE]      = "Vitesse rotation Yaw (rad/s)";
    g_Str[LS_ANG_INERTIA]   = "Inertie angulaire (0=instantané, 0.9=très progressif)";
    g_Str[LS_CAM_TILT]      = "Angle caméra (° vers le bas)";
    g_Str[LS_VISUAL_ROLL]   = "Roll visuel de la caméra";
    g_Str[LS_DRAG_H]        = "Drag linéaire horizontal";
    g_Str[LS_DRAG_V]        = "Drag linéaire vertical";
    g_Str[LS_QDRAG_H]       = "Drag quadratique horizontal";
    g_Str[LS_QDRAG_V]       = "Drag quadratique vertical";
    g_Str[LS_GRAVITY_EN]    = "Gravité activée";
    g_Str[LS_GRAVITY]       = "Force de gravité (u/s²)";
    g_Str[LS_DEADZONE]      = "Zone morte manette";
    g_Str[LS_MIN_H_EN]      = "Hauteur minimale (anti-sol)";
    g_Str[LS_MIN_H]         = "Valeur hauteur minimale";
    g_Str[LS_THR_MAX]       = "Throttle max (1.0=normal, 2.0=+100%)";
    g_Str[LS_THR_CURVE]     = "Courbe throttle (1.0=linéaire, 2.0=très creux)";
    g_Str[LS_MIN_THR_PCT]   = "Gaz minimum (%)";
    g_Str[LS_MOTOR_TAU]     = "Motor lag — constante de temps (0=instant)";
    g_Str[LS_GYRO]          = "Gyroscopic coupling (0=off, 1=réaliste)";
    g_Str[LS_DECIM_STEP]    = "Décimation injection (1 kf / N frames)";
    g_Str[LS_KF_FREQ]       = "Fréquence keyframes (Hz)";
    g_Str[LS_AXES_TAB]      = "Axes";
    g_Str[LS_INV_THROTTLE]  = "Inverser Throttle (stick G. Y)";
    g_Str[LS_INV_YAW]       = "Inverser Yaw (stick G. X)";
    g_Str[LS_INV_PITCH]     = "Inverser Pitch (stick D. Y)";
g_Str[LS_INV_ROLL]      = "Inverser Roll (stick D. X)";
    g_Str[LS_INV_VIS_ROLL]  = "Inverser Roll VISUEL";
    g_Str[LS_REWIND]        = "Rewind";
    g_Str[LS_EXPORT_SECT]   = "\\$888--- Export Mediatracker CLIP ---";
    g_Str[LS_EXPORT_STEP1]  = "Pour exporter un record en CLIP afin de le visualiser et le rendre dans Mediatracker :";
    g_Str[LS_EXPORT_STEP2]  = "1. Après l'enregistrement, clique sur le lien .csv dans le HUD";
    g_Str[LS_EXPORT_STEP3]  = "2. Le fichier est dans : C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
    g_Str[LS_EXPORT_BTN]    = "Ouvrir le dossier";
    g_Str[LS_EXPORT_LINK]   = " pour uploader votre .csv";
 
    if (g_Lang == 1) { // ── English ──────────────────────────
        g_Str[LS_OVERLAY_SECT]  = "\\$888── Stick overlay ──────────────────────────────";
        g_Str[LS_SHOW_OVERLAY]  = "Show stick overlay";
        g_Str[LS_SHOW_VALUES]   = "Show live values (pitch / roll / yaw)";
        g_Str[LS_SZ]            = "Box size";
        g_Str[LS_GAP]           = "Spacing";
        g_Str[LS_BG_ALPHA]      = "Background alpha";
        g_Str[LS_BORDER_ALPHA]  = "Border alpha";
        g_Str[LS_CROSS_ALPHA]   = "Cross alpha";
        g_Str[LS_DOT_SIZE]      = "Dot size";
        g_Str[LS_DOT_COLOR]     = "Dot color";
        g_Str[LS_PHYSICS_SECT]  = "\\$888── Current physics ─────────────────────────────";
        g_Str[LS_PRESET_SECT]   = "\\$888── Physics preset (Uncrashed) ──────────────────";
        g_Str[LS_BIND_HINT1]    = "\\$888Click a box then press the desired button/key.";
        g_Str[LS_BIND_HINT2]    = "Pad AND keyboard can be bound simultaneously. [×] = clear.";
        g_Str[LS_PAD_LBL]       = "Pad:";
        g_Str[LS_KEY_LBL]       = "Key:";
        g_Str[LS_PRESS_BTN]     = "< Press now... >";
        g_Str[LS_CP_HINT]       = "\\$888Checkpoint: 1st press = save, next presses = return. Long press = clear.";
        g_Str[LS_DRONE_ON]      = "FPV Drone active";
        g_Str[LS_CAM7]          = "\\$888(Cam 7 required)";
        g_Str[LS_SPEED_LBL]     = "Speed: ";
        g_Str[LS_THR_LBL]       = "Throttle: ";
        g_Str[LS_GRAV_LBL]      = "Grav: ";
        g_Str[LS_PRESETS_LBL]   = "\\$888Presets:";
        g_Str[LS_SAVE_LBL]      = "Save";
        g_Str[LS_LOAD_LBL]      = "Load";
        g_Str[LS_LANG_LBL]      = "Language:";
        g_Str[LS_HELP_TAB]      = "Help";
        g_Str[LS_HELP_WHAT]     = "FPV Drone is a camera plugin that simulates FPV drone physics in acrobatics mode. It works as soon as Camera 7 (free cam) is active.";
        g_Str[LS_HELP_REQ]      = "A gamepad is required.";
        g_Str[LS_HELP_CTRL]     = "Controls: Left stick Y = Throttle (up = full throttle) - Left stick X = Yaw - Right stick Y = Pitch - Right stick X = Roll.";
        g_Str[LS_HELP_REC]      = "Recording: short press on Record bind starts and stops recording. During recording, holding the Rewind bind makes the drone go back in time. You can keep your inputs active during rewind to smoothly resume the trajectory when released.";
        g_Str[LS_HELP_CP]       = "Checkpoint: first press saves current position, next press returns to that position. Long press clears the checkpoint.";
        g_Str[LS_HELP_PRESETS]  = "Presets: save and load your physics settings via the 3 slots in the HUD (bottom-left). Useful to switch between different flight feels.";
        g_Str[LS_STATUS_ON]     = "active";
        g_Str[LS_STATUS_OFF]    = "inactive";
        g_Str[LS_PHYSICS_TAB]   = "Physics";
        g_Str[LS_ENABLED]       = "Enabled";
        g_Str[LS_THRUST]        = "Motor thrust (u/s²)";
        g_Str[LS_TILT_RATE]     = "Pitch/Roll tilt rate (rad/s)";
        g_Str[LS_YAW_RATE]      = "Yaw rotation rate (rad/s)";
        g_Str[LS_ANG_INERTIA]   = "Angular inertia (0=instant, 0.9=very gradual)";
        g_Str[LS_CAM_TILT]      = "Camera tilt angle (° downward)";
        g_Str[LS_VISUAL_ROLL]   = "Visual camera roll";
        g_Str[LS_DRAG_H]        = "Horizontal linear drag";
        g_Str[LS_DRAG_V]        = "Vertical linear drag";
        g_Str[LS_QDRAG_H]       = "Horizontal quadratic drag";
        g_Str[LS_QDRAG_V]       = "Vertical quadratic drag";
        g_Str[LS_GRAVITY_EN]    = "Gravity enabled";
        g_Str[LS_GRAVITY]       = "Gravity force (u/s²)";
        g_Str[LS_DEADZONE]      = "Stick deadzone";
        g_Str[LS_MIN_H_EN]      = "Minimum height (anti-ground)";
        g_Str[LS_MIN_H]         = "Minimum height value";
        g_Str[LS_THR_MAX]       = "Max throttle (1.0=normal, 2.0=+100%)";
        g_Str[LS_THR_CURVE]     = "Throttle curve (1.0=linear, 2.0=very hollow)";
        g_Str[LS_MIN_THR_PCT]   = "Minimum throttle (%)";
        g_Str[LS_MOTOR_TAU]     = "Motor lag — time constant (0=instant)";
        g_Str[LS_GYRO]          = "Gyroscopic coupling (0=off, 1=realistic)";
        g_Str[LS_DECIM_STEP]    = "Injection decimation (1 kf / N frames)";
        g_Str[LS_KF_FREQ]       = "Keyframe frequency (Hz)";
        g_Str[LS_AXES_TAB]      = "Axes";
        g_Str[LS_INV_THROTTLE]  = "Invert Throttle (left stick Y)";
        g_Str[LS_INV_YAW]       = "Invert Yaw (left stick X)";
        g_Str[LS_INV_PITCH]     = "Invert Pitch (right stick Y)";
        g_Str[LS_INV_ROLL]      = "Invert Roll (right stick X)";
        g_Str[LS_INV_VIS_ROLL]  = "Invert VISUAL roll";
g_Str[LS_REWIND]        = "Rewind";
        g_Str[LS_EXPORT_SECT]   = "\\$888--- Export Mediatracker CLIP ---";
        g_Str[LS_EXPORT_STEP1]  = "To export a record as CLIP for viewing and rendering in Mediatracker:";
        g_Str[LS_EXPORT_STEP2]  = "1. After recording, click the .csv link in the HUD";
        g_Str[LS_EXPORT_STEP3]  = "2. The file is located at: C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
        g_Str[LS_EXPORT_BTN]    = "Open folder";
        g_Str[LS_EXPORT_LINK]   = " to upload your .csv";
 
     } else if (g_Lang == 2) { // ── Deutsch ──────────────────
        g_Str[LS_OVERLAY_SECT]  = "\\$888── Stick-Overlay ─────────────────────────────";
        g_Str[LS_SHOW_OVERLAY]  = "Stick-Overlay anzeigen";
        g_Str[LS_SHOW_VALUES]   = "Livewerte anzeigen (Pitch / Roll / Yaw)";
        g_Str[LS_SZ]            = "Boxgröße";
        g_Str[LS_GAP]           = "Abstand";
        g_Str[LS_BG_ALPHA]      = "Hintergrund Alpha";
        g_Str[LS_BORDER_ALPHA]  = "Rand Alpha";
        g_Str[LS_CROSS_ALPHA]   = "Kreuz Alpha";
        g_Str[LS_DOT_SIZE]      = "Punktgröße";
        g_Str[LS_DOT_COLOR]     = "Punktfarbe";
        g_Str[LS_PHYSICS_SECT]  = "\\$888── Aktuelle Physik ────────────────────────────";
        g_Str[LS_PRESET_SECT]   = "\\$888── Physik-Preset (Uncrashed) ────────────────";
        g_Str[LS_BIND_HINT1]    = "\\$888Klicke eine Box an, dann drücke die gewünschte Taste.";
        g_Str[LS_BIND_HINT2]    = "Pad UND Tastatur können gleichzeitig belegt werden. [×] = löschen.";
        g_Str[LS_RECORD]        = "Aufnahme";
        g_Str[LS_PAD_LBL]       = "Pad:";
        g_Str[LS_KEY_LBL]       = "Taste:";
        g_Str[LS_PRESS_BTN]     = "< Drücken... >";
        g_Str[LS_CP_HINT]       = "\\$888Checkpoint: 1. Druck = speichern, folgende = zurückkehren. Lang = löschen.";
        g_Str[LS_DRONE_ON]      = "FPV Drone aktiv";
        g_Str[LS_CAM7]          = "\\$888(Kamera 7 erforderlich)";
        g_Str[LS_SPEED_LBL]     = "Geschw.: ";
        g_Str[LS_THR_LBL]       = "Gas: ";
        g_Str[LS_GRAV_LBL]      = "Schwerkraft: ";
        g_Str[LS_ON_LBL]        = "AN";
        g_Str[LS_OFF_LBL]       = "AUS";
        g_Str[LS_PRESETS_LBL]   = "\\$888Presets:";
        g_Str[LS_SAVE_LBL]      = "Speichern";
        g_Str[LS_LOAD_LBL]      = "Laden";
        g_Str[LS_LANG_LBL]      = "Sprache:";
        g_Str[LS_HELP_TAB]      = "Hilfe";
        g_Str[LS_BINDS_TAB]     = "Binds";
        g_Str[LS_HELP_WHAT]     = "FPV Drone ist ein Kamera-Plugin, das FPV-Drohnenphysik im Akrobatik-Modus simuliert. Es funktioniert, sobald Kamera 7 (freie Kamera) aktiv ist.";
        g_Str[LS_HELP_REQ]      = "Ein Gamepad wird benötigt, um die Drohne zu fliegen.";
        g_Str[LS_HELP_CTRL]     = "Steuerung: Linker Stick Y = Gas (oben = Vollgas) - Linker Stick X = Gieren - Rechter Stick Y = Neigen - Rechter Stick X = Rollen.";
        g_Str[LS_HELP_REC]      = "Aufnahme: Ein kurzer Druck auf den Record-Bind startet und stoppt die Aufnahme. Während der Aufnahme hältst du den Rewind-Bind gedrückt, um in der Zeit zurückzugehen. Du kannst deine Inputs während des Rewinds aktiv halten, um beim Loslassen nahtlos fortzufahren.";
        g_Str[LS_HELP_CP]       = "Checkpoint: Der erste Druck speichert die aktuelle Position, der nächste Druck kehrt zu dieser Position zurück. Ein langer Druck löscht den Checkpoint.";
        g_Str[LS_HELP_PRESETS]  = "Presets: Speichere und lade Physikeinstellungen mit den 3 Slots im HUD (unten links). Nützlich, um zwischen verschiedenen Fluggefühlen zu wechseln.";
        g_Str[LS_STATUS_ON]     = "aktiv";
        g_Str[LS_STATUS_OFF]    = "inaktiv";
        g_Str[LS_PHYSICS_TAB]   = "Physik";
        g_Str[LS_ENABLED]       = "Aktiviert";
        g_Str[LS_THRUST]        = "Motorschub (u/s²)";
        g_Str[LS_TILT_RATE]     = "Pitch/Roll-Rate (rad/s)";
        g_Str[LS_YAW_RATE]      = "Gier-Rate (rad/s)";
        g_Str[LS_ANG_INERTIA]   = "Winkelträgheit (0=sofort, 0.9=sehr träge)";
        g_Str[LS_CAM_TILT]      = "Kamerawinkel (° nach unten)";
        g_Str[LS_VISUAL_ROLL]   = "Visuelles Kamera-Roll";
        g_Str[LS_DRAG_H]        = "Horizontaler linearer Widerstand";
        g_Str[LS_DRAG_V]        = "Vertikaler linearer Widerstand";
        g_Str[LS_QDRAG_H]       = "Horizontaler quadratischer Widerstand";
        g_Str[LS_QDRAG_V]       = "Vertikaler quadratischer Widerstand";
        g_Str[LS_GRAVITY_EN]    = "Schwerkraft aktiviert";
        g_Str[LS_GRAVITY]       = "Schwerkraft (u/s²)";
        g_Str[LS_DEADZONE]      = "Stick-Totzone";
        g_Str[LS_MIN_H_EN]      = "Mindesthöhe (Anti-Boden)";
        g_Str[LS_MIN_H]         = "Mindesthöhe Wert";
        g_Str[LS_THR_MAX]       = "Max Gas (1.0=normal, 2.0=+100%)";
        g_Str[LS_THR_CURVE]     = "Gaskurve (1.0=linear, 2.0=sehr hohl)";
        g_Str[LS_MIN_THR_PCT]   = "Mindestgas (%)";
        g_Str[LS_MOTOR_TAU]     = "Motor Lag — Zeitkonstante (0=sofort)";
        g_Str[LS_GYRO]          = "Gyro-Kopplung (0=aus, 1=realistisch)";
        g_Str[LS_DECIM_STEP]    = "Injektions-Dezimierung (1 kf / N Frames)";
        g_Str[LS_KF_FREQ]       = "Keyframe-Frequenz (Hz)";
        g_Str[LS_AXES_TAB]      = "Achsen";
        g_Str[LS_INV_THROTTLE]  = "Gas umkehren (linker Stick Y)";
        g_Str[LS_INV_YAW]       = "Gieren umkehren (linker Stick X)";
        g_Str[LS_INV_PITCH]     = "Neigen umkehren (rechter Stick Y)";
g_Str[LS_INV_ROLL]      = "Rollen umkehren (rechter Stick X)";
        g_Str[LS_INV_VIS_ROLL]  = "Visuelles Roll umkehren";
        g_Str[LS_REWIND]        = "Rewind";
        g_Str[LS_EXPORT_SECT]   = "\\$888--- Export Mediatracker CLIP ---";
        g_Str[LS_EXPORT_STEP1]  = "Um eine Aufnahme als CLIP zum Ansehen und Rendern in Mediatracker zu exportieren:";
        g_Str[LS_EXPORT_STEP2]  = "1. Nach der Aufnahme klicke auf den .csv-Link im HUD";
        g_Str[LS_EXPORT_STEP3]  = "2. Die Datei befindet sich unter: C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
        g_Str[LS_EXPORT_BTN]    = "Ordner öffnen";
        g_Str[LS_EXPORT_LINK]   = " um Ihre .csv hochzuladen";
 
     } else if (g_Lang == 3) { // ── Español ──────────────────
        g_Str[LS_OVERLAY_SECT]  = "\\$888── Overlay sticks ──────────────────────────";
        g_Str[LS_SHOW_OVERLAY]  = "Mostrar overlay sticks";
        g_Str[LS_SHOW_VALUES]   = "Mostrar valores (pitch / roll / yaw)";
        g_Str[LS_POS_X]         = "Posición X";
        g_Str[LS_POS_Y]         = "Posición Y";
        g_Str[LS_SZ]            = "Tamaño cajas";
        g_Str[LS_GAP]           = "Separación";
        g_Str[LS_BG_ALPHA]      = "Alpha fondo";
        g_Str[LS_BORDER_ALPHA]  = "Alpha borde";
        g_Str[LS_CROSS_ALPHA]   = "Alpha cruz";
        g_Str[LS_DOT_SIZE]      = "Tamaño punto";
        g_Str[LS_DOT_COLOR]     = "Color punto";
        g_Str[LS_PHYSICS_SECT]  = "\\$888── Física actual ───────────────────────────";
        g_Str[LS_PRESET_SECT]   = "\\$888── Preset física (Uncrashed) ──────────────";
        g_Str[LS_BIND_HINT1]    = "\\$888Haz clic en una caja y pulsa el botón/tecla deseado.";
        g_Str[LS_BIND_HINT2]    = "Mando Y teclado pueden asignarse a la vez. [×] = borrar.";
        g_Str[LS_RECORD]        = "Grabar";
        g_Str[LS_CHECKPOINT]    = "Punto de control";
        g_Str[LS_PAD_LBL]       = "Mando:";
        g_Str[LS_KEY_LBL]       = "Tecla:";
        g_Str[LS_PRESS_BTN]     = "< Pulsar... >";
        g_Str[LS_CP_HINT]       = "\\$888Punto de control: 1ª pulsación = guardar, siguientes = volver. Largo = borrar.";
        g_Str[LS_DRONE_ON]      = "FPV Drone activo";
        g_Str[LS_CAM7]          = "\\$888(Cámara 7 requerida)";
        g_Str[LS_SPEED_LBL]     = "Velocidad: ";
        g_Str[LS_THR_LBL]       = "Gas: ";
        g_Str[LS_GRAV_LBL]      = "Grav: ";
        g_Str[LS_PRESETS_LBL]   = "\\$888Presets:";
        g_Str[LS_SAVE_LBL]      = "Guardar";
        g_Str[LS_LOAD_LBL]      = "Cargar";
        g_Str[LS_LANG_LBL]      = "Idioma:";
        g_Str[LS_HELP_TAB]      = "Ayuda";
        g_Str[LS_BINDS_TAB]     = "Controles";
        g_Str[LS_HELP_WHAT]     = "FPV Drone es un plugin de cámara que simula la física de un dron FPV en modo acrobático. Funciona tan pronto como la Cámara 7 (cámara libre) está activa.";
        g_Str[LS_HELP_REQ]      = "Se requiere un mando para pilotar el dron.";
        g_Str[LS_HELP_CTRL]     = "Controles: Stick izquierdo Y = Gas (arriba = gas completo) - Stick izquierdo X = Giro - Stick derecho Y = Cabeceo - Stick derecho X = Balanceo.";
        g_Str[LS_HELP_REC]      = "Grabación: Una pulsación corta en Record inicia y detiene la grabación. Durante la grabación, mantener Rewind hace que el dron vuelva atrás en el tiempo. Puedes mantener tus inputs activos durante el rewind para reanudar la trayectoria suavemente al soltar.";
        g_Str[LS_HELP_CP]       = "Checkpoint: La primera pulsación guarda la posición actual, la próxima pulsación te devuelve a esa posición. Una pulsación larga borra el checkpoint.";
        g_Str[LS_HELP_PRESETS]  = "Presets: Guarda y carga tus ajustes de física con los 3 slots en el HUD (abajo a la izquierda). Útil para alternar entre diferentes sensaciones de vuelo.";
        g_Str[LS_STATUS_ON]     = "activo";
        g_Str[LS_STATUS_OFF]    = "inactivo";
        g_Str[LS_PHYSICS_TAB]   = "Física";
        g_Str[LS_ENABLED]       = "Activado";
        g_Str[LS_THRUST]        = "Empuje del motor (u/s²)";
        g_Str[LS_TILT_RATE]     = "Vel. inclinación Pitch/Roll (rad/s)";
        g_Str[LS_YAW_RATE]      = "Vel. rotación Yaw (rad/s)";
        g_Str[LS_ANG_INERTIA]   = "Inercia angular (0=instantáneo, 0.9=muy gradual)";
        g_Str[LS_CAM_TILT]      = "Ángulo cámara (° hacia abajo)";
        g_Str[LS_VISUAL_ROLL]   = "Roll visual de la cámara";
        g_Str[LS_DRAG_H]        = "Resistencia horizontal lineal";
        g_Str[LS_DRAG_V]        = "Resistencia vertical lineal";
        g_Str[LS_QDRAG_H]       = "Resistencia horizontal cuadrática";
        g_Str[LS_QDRAG_V]       = "Resistencia vertical cuadrática";
        g_Str[LS_GRAVITY_EN]    = "Gravedad activada";
        g_Str[LS_GRAVITY]       = "Fuerza de gravedad (u/s²)";
        g_Str[LS_DEADZONE]      = "Zona muerta del mando";
        g_Str[LS_MIN_H_EN]      = "Altura mínima (anti-suelo)";
        g_Str[LS_MIN_H]         = "Valor altura mínima";
        g_Str[LS_THR_MAX]       = "Gas máximo (1.0=normal, 2.0=+100%)";
        g_Str[LS_THR_CURVE]     = "Curva gas (1.0=lineal, 2.0=muy cóncavo)";
        g_Str[LS_MIN_THR_PCT]   = "Gas mínimo (%)";
        g_Str[LS_MOTOR_TAU]     = "Lag motor — constante de tiempo (0=instantáneo)";
        g_Str[LS_GYRO]          = "Acoplamiento giroscópico (0=no, 1=realista)";
        g_Str[LS_DECIM_STEP]    = "Decimación inyección (1 kf / N frames)";
        g_Str[LS_KF_FREQ]       = "Frecuencia keyframes (Hz)";
        g_Str[LS_AXES_TAB]      = "Ejes";
        g_Str[LS_INV_THROTTLE]  = "Invertir Gas (stick izq. Y)";
        g_Str[LS_INV_YAW]       = "Invertir Giro (stick izq. X)";
        g_Str[LS_INV_PITCH]     = "Invertir Cabeceo (stick der. Y)";
        g_Str[LS_INV_ROLL]      = "Invertir Balanceo (stick der. X)";
        g_Str[LS_INV_VIS_ROLL]  = "Invertir Roll VISUAL";
g_Str[LS_REWIND]        = "Rewind";
        g_Str[LS_EXPORT_SECT]   = "\\$888--- Export Mediatracker CLIP ---";
        g_Str[LS_EXPORT_STEP1]  = "Para exportar un record como CLIP para visualizar y renderizar en Mediatracker:";
        g_Str[LS_EXPORT_STEP2]  = "1. Después de la grabación, haz clic en el enlace .csv en el HUD";
        g_Str[LS_EXPORT_STEP3]  = "2. El archivo está en: C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
        g_Str[LS_EXPORT_BTN]    = "Abrir carpeta";
        g_Str[LS_EXPORT_LINK]   = " para subir tu .csv";
 
     } else if (g_Lang == 4) { // ── Русский ──────────────────
        g_Str[LS_OVERLAY_SECT]  = "\\$888── Оверлей стиков ──────────────────────────";
        g_Str[LS_SHOW_OVERLAY]  = "Показать оверлей стиков";
        g_Str[LS_SHOW_VALUES]   = "Показать значения (тангаж / крен / рыскание)";
        g_Str[LS_POS_X]         = "Позиция X";
        g_Str[LS_POS_Y]         = "Позиция Y";
        g_Str[LS_SZ]            = "Размер блоков";
        g_Str[LS_GAP]           = "Промежуток";
        g_Str[LS_BG_ALPHA]      = "Прозрачность фона";
        g_Str[LS_BORDER_ALPHA]  = "Прозрачность рамки";
        g_Str[LS_CROSS_ALPHA]   = "Прозрачность крестика";
        g_Str[LS_DOT_SIZE]      = "Размер точки";
        g_Str[LS_DOT_COLOR]     = "Цвет точки";
        g_Str[LS_PHYSICS_SECT]  = "\\$888── Текущая физика ──────────────────────────";
        g_Str[LS_PRESET_SECT]   = "\\$888── Пресет физики (Uncrashed) ─────────────";
        g_Str[LS_BIND_HINT1]    = "\\$888Нажмите на поле, затем нажмите нужную кнопку/клавишу.";
        g_Str[LS_BIND_HINT2]    = "Геймпад И клавиатура могут быть назначены одновременно. [×] = очистить.";
        g_Str[LS_RECORD]        = "Запись";
        g_Str[LS_RESPAWN]       = "Респавн";
        g_Str[LS_CHECKPOINT]    = "Контр. точка";
        g_Str[LS_PAD_LBL]       = "Геймпад:";
        g_Str[LS_KEY_LBL]       = "Клавиша:";
        g_Str[LS_PRESS_BTN]     = "< Нажмите... >";
        g_Str[LS_CP_HINT]       = "\\$888КТ: 1-е нажатие = сохранить, следующие = вернуться. Долгое = удалить.";
        g_Str[LS_DRONE_ON]      = "FPV Drone активен";
        g_Str[LS_CAM7]          = "\\$888(нужна Камера 7)";
        g_Str[LS_SPEED_LBL]     = "Скорость: ";
        g_Str[LS_THR_LBL]       = "Газ: ";
        g_Str[LS_GRAV_LBL]      = "Гравит.: ";
        g_Str[LS_ON_LBL]        = "ВКЛ";
        g_Str[LS_OFF_LBL]       = "ВЫКЛ";
        g_Str[LS_PRESETS_LBL]   = "\\$888Пресеты:";
        g_Str[LS_SAVE_LBL]      = "Сохр.";
        g_Str[LS_LOAD_LBL]      = "Загр.";
        g_Str[LS_LANG_LBL]      = "Язык:";
        g_Str[LS_HELP_TAB]      = "Помощь";
        g_Str[LS_MISC_TAB]      = "Разное";
        g_Str[LS_BINDS_TAB]     = "Кнопки";
        g_Str[LS_HELP_WHAT]     = "FPV Drone — это плагин для камеры, симулирующий физику FPV-дрона в акробатическом режиме. Работает, как только активируется Камера 7 (свободная камера).";
        g_Str[LS_HELP_REQ]      = "Для управления дроном требуется геймпад.";
        g_Str[LS_HELP_CTRL]     = "Управление: Левый стик Y = Газ (вверх = полный газ) - Левый стик X = Рыскание - Правый стик Y = Тангаж - Правый стик X = Крен.";
        g_Str[LS_HELP_REC]      = "Запись: Короткое нажатие на кнопку Record начинает и останавливает запись. Во время записи удержание кнопки Rewind возвращает дрон назад во времени. Вы можете удерживать стики активными во время перемотки, чтобы плавно продолжить полет при отпускании.";
        g_Str[LS_HELP_CP]       = "Контрольная точка (Checkpoint): Первое нажатие сохраняет текущую позицию, следующее возвращает к ней. Долгое нажатие удаляет контрольную точку.";
        g_Str[LS_HELP_PRESETS]  = "Пресеты: Сохраняйте и загружайте настройки физики в 3 слота в HUD (снизу слева). Полезно для переключения между разными стилями полета.";
        g_Str[LS_STATUS_ON]     = "активен";
        g_Str[LS_STATUS_OFF]    = "неактивен";
        g_Str[LS_PHYSICS_TAB]   = "Физика";
        g_Str[LS_ENABLED]       = "Включён";
        g_Str[LS_THRUST]        = "Тяга мотора (u/s²)";
        g_Str[LS_TILT_RATE]     = "Скорость наклона Pitch/Roll (рад/с)";
        g_Str[LS_YAW_RATE]      = "Скорость рыскания (рад/с)";
        g_Str[LS_ANG_INERTIA]   = "Угловая инерция (0=мгновенно, 0.9=очень постепенно)";
        g_Str[LS_CAM_TILT]      = "Угол камеры (° вниз)";
        g_Str[LS_VISUAL_ROLL]   = "Визуальный крен камеры";
        g_Str[LS_DRAG_H]        = "Горизонтальное линейное сопротивление";
        g_Str[LS_DRAG_V]        = "Вертикальное линейное сопротивление";
        g_Str[LS_QDRAG_H]       = "Горизонтальное квадратичное сопротивление";
        g_Str[LS_QDRAG_V]       = "Вертикальное квадратичное сопротивление";
        g_Str[LS_GRAVITY_EN]    = "Гравитация включена";
        g_Str[LS_GRAVITY]       = "Сила гравитации (u/s²)";
        g_Str[LS_DEADZONE]      = "Мёртвая зона стика";
        g_Str[LS_MIN_H_EN]      = "Минимальная высота (анти-земля)";
        g_Str[LS_MIN_H]         = "Значение минимальной высоты";
        g_Str[LS_THR_MAX]       = "Макс. газ (1.0=норм., 2.0=+100%)";
        g_Str[LS_THR_CURVE]     = "Кривая газа (1.0=линейная, 2.0=вогнутая)";
        g_Str[LS_MIN_THR_PCT]   = "Минимальный газ (%)";
        g_Str[LS_MOTOR_TAU]     = "Задержка мотора — константа времени (0=мгновенно)";
        g_Str[LS_GYRO]          = "Гироскопическая связь (0=выкл, 1=реалистично)";
        g_Str[LS_DECIM_STEP]    = "Децимация инъекции (1 kf / N кадров)";
        g_Str[LS_KF_FREQ]       = "Частота keyframes (Гц)";
        g_Str[LS_AXES_TAB]      = "Оси";
        g_Str[LS_INV_THROTTLE]  = "Инвертировать газ (лев. стик Y)";
        g_Str[LS_INV_YAW]       = "Инвертировать рыскание (лев. стик X)";
        g_Str[LS_INV_PITCH]     = "Инвертировать тангаж (пр. стик Y)";
        g_Str[LS_INV_ROLL]      = "Инвертировать крен (пр. стик X)";
g_Str[LS_INV_VIS_ROLL]  = "Инвертировать ВИЗУАЛЬНЫЙ крен";
        g_Str[LS_REWIND]        = "Перемотка назад";
        g_Str[LS_EXPORT_SECT]   = "\\$888--- Экспорт CLIP Mediatracker ---";
        g_Str[LS_EXPORT_STEP1]  = "Чтобы экспортировать запись в CLIP для просмотра и рендеринга в Mediatracker:";
        g_Str[LS_EXPORT_STEP2]  = "1. После записи, кликните на ссылку .csv в HUD";
        g_Str[LS_EXPORT_STEP3]  = "2. Файл расположен по пути: C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
        g_Str[LS_EXPORT_BTN]    = "Открыть папку";
        g_Str[LS_EXPORT_LINK]   = " для загрузки вашего .csv";
 
     } else if (g_Lang == 5) { // ── Polski ───────────────────
        g_Str[LS_OVERLAY_SECT]  = "\\$888── Nakładka drążków ────────────────────────";
        g_Str[LS_SHOW_OVERLAY]  = "Pokaż nakładkę drążków";
        g_Str[LS_SHOW_VALUES]   = "Pokaż wartości (pitch / roll / yaw)";
        g_Str[LS_POS_X]         = "Pozycja X";
        g_Str[LS_POS_Y]         = "Pozycja Y";
        g_Str[LS_SZ]            = "Rozmiar pól";
        g_Str[LS_GAP]           = "Odstęp";
        g_Str[LS_BG_ALPHA]      = "Przezroczystość tła";
        g_Str[LS_BORDER_ALPHA]  = "Przezroczystość ramki";
        g_Str[LS_CROSS_ALPHA]   = "Przezroczystość krzyżyka";
        g_Str[LS_DOT_SIZE]      = "Rozmiar kropki";
        g_Str[LS_DOT_COLOR]     = "Kolor kropki";
        g_Str[LS_PHYSICS_SECT]  = "\\$888── Aktualna fizyka ─────────────────────────";
        g_Str[LS_PRESET_SECT]   = "\\$888── Preset fizyki (Uncrashed) ──────────────";
        g_Str[LS_BIND_HINT1]    = "\\$888Kliknij pole, a następnie naciśnij żądany przycisk/klawisz.";
        g_Str[LS_BIND_HINT2]    = "Pad I klawiatura mogą być przypisane jednocześnie. [×] = usuń.";
        g_Str[LS_RECORD]        = "Nagrywanie";
        g_Str[LS_RESPAWN]       = "Respawn";
        g_Str[LS_CHECKPOINT]    = "Punkt kontrolny";
        g_Str[LS_PAD_LBL]       = "Pad:";
        g_Str[LS_KEY_LBL]       = "Klawisz:";
        g_Str[LS_PRESS_BTN]     = "< Naciśnij... >";
        g_Str[LS_CP_HINT]       = "\\$888PK: 1. naciśnięcie = zapisz, następne = wróć. Długie = usuń.";
        g_Str[LS_DRONE_ON]      = "FPV Drone aktywny";
        g_Str[LS_CAM7]          = "\\$888(wymagana Kamera 7)";
        g_Str[LS_SPEED_LBL]     = "Prędkość: ";
        g_Str[LS_THR_LBL]       = "Gaz: ";
        g_Str[LS_GRAV_LBL]      = "Grawitacja: ";
        g_Str[LS_ON_LBL]        = "WŁ";
        g_Str[LS_OFF_LBL]       = "WYŁ";
        g_Str[LS_PRESETS_LBL]   = "\\$888Presety:";
        g_Str[LS_SAVE_LBL]      = "Zapisz";
        g_Str[LS_LOAD_LBL]      = "Wczytaj";
        g_Str[LS_LANG_LBL]      = "Język:";
        g_Str[LS_HELP_TAB]      = "Pomoc";
        g_Str[LS_MISC_TAB]      = "Różne";
        g_Str[LS_BINDS_TAB]     = "Przyciski";
        g_Str[LS_HELP_WHAT]     = "FPV Drone to wtyczka kamery symulująca fizykę drona FPV w trybie akrobatycznym. Działa, gdy tylko Kamera 7 (wolna kamera) jest aktywna.";
        g_Str[LS_HELP_REQ]      = "Do sterowania dronem wymagany jest gamepad.";
        g_Str[LS_HELP_CTRL]     = "Sterowanie: Lewy drążek Y = Gaz (w górę = pełny gaz) - Lewy drążek X = Odchylenie (Yaw) - Prawy drążek Y = Pochylenie (Pitch) - Prawy drążek X = Przechylenie (Roll).";
        g_Str[LS_HELP_REC]      = "Nagrywanie: Krótkie naciśnięcie przypisanego przycisku Record uruchamia i zatrzymuje nagrywanie. Podczas nagrywania przytrzymanie przycisku Rewind sprawia, że dron cofa się w czasie. Możesz trzymać aktywne wejścia podczas przewijania, aby płynnie wznowić lot po zwolnieniu.";
        g_Str[LS_HELP_CP]       = "Punkt kontrolny (Checkpoint): Pierwsze naciśnięcie zapisuje bieżącą pozycję, następne wraca do tej pozycji. Długie naciśnięcie usuwa punkt kontrolny.";
        g_Str[LS_HELP_PRESETS]  = "Presety: Zapisuj i wczytuj ustawienia fizyki przez 3 sloty w HUD (lewy dół). Przydatne do przełączania się między różnymi odczuciami z lotu.";
        g_Str[LS_STATUS_ON]     = "aktywny";
        g_Str[LS_STATUS_OFF]    = "nieaktywny";
        g_Str[LS_PHYSICS_TAB]   = "Fizyka";
        g_Str[LS_ENABLED]       = "Włączony";
        g_Str[LS_THRUST]        = "Ciąg silnika (u/s²)";
        g_Str[LS_TILT_RATE]     = "Prędkość pochylenia Pitch/Roll (rad/s)";
        g_Str[LS_YAW_RATE]      = "Prędkość obrotu Yaw (rad/s)";
        g_Str[LS_ANG_INERTIA]   = "Bezwładność kątowa (0=natychmiastowa, 0.9=bardzo stopniowa)";
        g_Str[LS_CAM_TILT]      = "Kąt kamery (° w dół)";
        g_Str[LS_VISUAL_ROLL]   = "Wizualny przechył kamery";
        g_Str[LS_DRAG_H]        = "Poziomy opór liniowy";
        g_Str[LS_DRAG_V]        = "Pionowy opór liniowy";
        g_Str[LS_QDRAG_H]       = "Poziomy opór kwadratowy";
        g_Str[LS_QDRAG_V]       = "Pionowy opór kwadratowy";
        g_Str[LS_GRAVITY_EN]    = "Grawitacja włączona";
        g_Str[LS_GRAVITY]       = "Siła grawitacji (u/s²)";
        g_Str[LS_DEADZONE]      = "Martwa strefa drążka";
        g_Str[LS_MIN_H_EN]      = "Minimalna wysokość (anty-ziemia)";
        g_Str[LS_MIN_H]         = "Wartość minimalnej wysokości";
        g_Str[LS_THR_MAX]       = "Maks. gaz (1.0=normalne, 2.0=+100%)";
        g_Str[LS_THR_CURVE]     = "Krzywa gazu (1.0=liniowa, 2.0=wklęsła)";
        g_Str[LS_MIN_THR_PCT]   = "Minimalny gaz (%)";
        g_Str[LS_MOTOR_TAU]     = "Opóźnienie silnika — stała czasowa (0=natychmiast)";
        g_Str[LS_GYRO]          = "Sprzężenie żyroskopowe (0=wył, 1=realistyczne)";
        g_Str[LS_DECIM_STEP]    = "Decymacja iniekcji (1 kf / N klatek)";
        g_Str[LS_KF_FREQ]       = "Częstotliwość keyframes (Hz)";
        g_Str[LS_AXES_TAB]      = "Osie";
        g_Str[LS_INV_THROTTLE]  = "Odwróć Gaz (lewy drążek Y)";
        g_Str[LS_INV_YAW]       = "Odwróć Obrót (lewy drążek X)";
        g_Str[LS_INV_PITCH]     = "Odwróć Pochylenie (prawy drążek Y)";
        g_Str[LS_INV_ROLL]      = "Odwróć Przechylenie (prawy drążek X)";
        g_Str[LS_INV_VIS_ROLL]  = "Odwróć WIZUALNY przechył";
        g_Str[LS_REWIND]        = "Rewind";
        g_Str[LS_EXPORT_SECT]   = "\\$888--- Eksport CLIP Mediatracker ---";
        g_Str[LS_EXPORT_STEP1]  = "Aby wyeksportować nagranie jako CLIP do podglądu i renderowania w Mediatracker:";
        g_Str[LS_EXPORT_STEP2]  = "1. Po nagraniu kliknij link .csv w HUD";
        g_Str[LS_EXPORT_STEP3]  = "2. Plik znajduje się w: C:\\Users\\name\\OpenplanetNext\\PluginStorage\\FPVDrone\\trajectory.csv";
        g_Str[LS_EXPORT_BTN]    = "Otwórz folder";
        g_Str[LS_EXPORT_LINK]   = " aby przesłać swój .csv";
    }
}

void ApplyUncrashed5FreestylePreset() {
    // Sources for claims used in this mapping:
    // - Uncrashed categories and realism goal (official): https://store.steampowered.com/app/1682970
    // - 5" freestyle context, 6S context, min throttle default=5, throttle curve default=1.0,
    //   and advanced physics parameter meaning: https://steamcommunity.com/app/1682970/discussions/0/3084394448734477548/
    // - Exact target numeric profile values are copied from the user-provided screenshot for
    //   "5\" freestyle" in this task (camera=30, kv=1750, cells=6, pitch=3.6, weight=730,
    //   min throttle=5.5, gravity=9.82, instant power=0.45, air friction=0.40, air grip=0.40,
    //   relative air speed=1.00, low/mid/high throttle=1.00).
    //
    // Mapping note:
    // Uncrashed and this plugin do not expose the same parameter set. We map all plugin settings
    // with deterministic rules and clamp to plugin ranges where needed.

    // Core mode
    S_Enabled = true;

    // Camera: Uncrashed "FPV camera angle 30°" -> this plugin uses downward tilt sign convention.
    S_CameraTilt = -30.0f;
    S_VisualRoll = true;
    S_InvVisualRoll = false;

    // Rates / inertia:
    // "PID behavior: Perfect" -> no added angular lag in this plugin.
    S_AngInertia = 0.0f;
    // No explicit Uncrashed rate value in the screenshot; keep existing pilot-feel defaults.
    S_TiltRate = 10.0f;
    S_YawRate  = 10.0f;
    S_GyroCoeff = 1.0f;

    // Gravity and thrust are converted to this plugin's environment units.
    // In this plugin, "Earth-like gravity" default is 50.0 (existing plugin baseline).
    const float kEnvGravity = 50.0f;
    const float gravityUncrashed = 9.82f;
    const float thrustKgf = 7.199f;
    const float droneMassKg = 0.730f;
    const float thrustToWeight = thrustKgf / droneMassKg; // 9.8616...

    S_GravityEnabled = true;
    S_Gravity = kEnvGravity * (gravityUncrashed / 9.82f); // = 50.0 exactly for Earth gravity.
    S_Thrust = S_Gravity * thrustToWeight;                // preserve Uncrashed thrust-to-weight ratio.

    // Air friction/grip in Uncrashed are not the same equations as this plugin's drag model.
    // Keep plugin drag coefficients at known-stable calibrated values.
    S_Drag = 0.024f;
    S_DragY = 0.032f;
    S_QuadDrag = 0.0f;
    S_QuadDragY = 0.0f;

    // Throttle response: low/mid/high are 1.00 in screenshot => linear curve in this plugin.
    S_ThrottleCurve = 1.0f;
    S_ThrottleMax = 1.0f;
    S_MinThrottlePct = 5.5f; // exact screenshot value.
    // Instant power 0.45 -> map to first-order motor alpha at 60 Hz: alpha=dt/tau, dt=1/60.
    // tau = (1/60) / 0.45 = 0.037037...
    S_MotorTau = 0.037f;

    // Input / axes
    S_Deadzone = 0.0f;
    S_InvThrottle = false;
    S_InvYaw = false;
    S_InvPitch = false;
    S_InvRoll = false;

    // Safety / QoL
    S_UseMinHeight = true;
    S_MinHeight = 9.0f;

    // Trajectory settings
    S_Rec = false;
    S_Play = false;
    S_KfFreq = 1;

    // Concrete self-check report (exact values in this environment after conversion).
    float twNow = (S_Gravity > 0.0001f) ? (S_Thrust / S_Gravity) : 0.0f;
    float hoverNow = (S_Thrust > 0.0001f) ? Math::Sqrt(S_Gravity / S_Thrust) : 0.0f;
    g_UncrashedTestReport =
        "CHECK: cam=" + Text::Format("%.1f", Math::Abs(S_CameraTilt))
        + " deg, g=" + Text::Format("%.2f", S_Gravity)
        + ", thrust=" + Text::Format("%.2f", S_Thrust)
        + ", T/W=" + Text::Format("%.3f", twNow)
        + ", hoverThr~" + Text::Format("%.3f", hoverNow)
        + ", minThr=" + Text::Format("%.1f", S_MinThrottlePct)
        + ", tau=" + Text::Format("%.3f", S_MotorTau);
    g_UncrashedPresetStatus =
        "Preset Uncrashed 5\" applique (conversion environnement): gravite Earth->50, T/W conserve depuis 7.199kg/730g.";
}

// Retourne le nouvel index de bind. slot : 0=RecPad,1=RecKey,2=RespawnPad,3=RespawnKey
int BindBtn(const string &in id, const array<string>@ lbls, int cur, int slot) {
    bool cap = (g_CapturingBind == slot);
    string lbl = cap ? T(LS_PRESS_BTN) : lbls[Math::Clamp(cur, 0, int(lbls.Length) - 1)];
    if (cap) UI::PushStyleColor(UI::Col::Button, vec4(0.7f, 0.4f, 0.0f, 1.0f));
    if (UI::Button(lbl + "##" + id)) {
        g_CapturingBind = (g_CapturingBind == slot) ? -1 : slot;
    }
    if (cap) UI::PopStyleColor(1);
    UI::SameLine();
    if (UI::Button("x##" + id)) { cur = 0; g_CapturingBind = -1; SaveBinds(); }
    return cur;
}

void RenderSettings() {
    UI::BeginTabBar("##fpvtabs");
    {
        // ── Onglet Physique ──────────────────────────────────────
        if (UI::BeginTabItem(T(LS_PHYSICS_TAB))) {
            UI::Text("");
            bool phyChanged = false;

            bool nEn = UI::Checkbox(T(LS_ENABLED), S_Enabled);
            if (nEn != S_Enabled) { S_Enabled = nEn; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_THRUST)); UI::SetNextItemWidth(300);
            float nThrust = UI::SliderFloat("##thrust", S_Thrust, 10.0f, 1000.0f);
            if (nThrust != S_Thrust) { S_Thrust = nThrust; phyChanged = true; }

            UI::Text(T(LS_TILT_RATE)); UI::SetNextItemWidth(300);
            float nTR = UI::SliderFloat("##tr", S_TiltRate, 0.2f, 20.0f);
            if (nTR != S_TiltRate) { S_TiltRate = nTR; phyChanged = true; }

            UI::Text(T(LS_YAW_RATE)); UI::SetNextItemWidth(300);
            float nYR = UI::SliderFloat("##yr", S_YawRate, 0.1f, 20.0f);
            if (nYR != S_YawRate) { S_YawRate = nYR; phyChanged = true; }

            UI::Text(T(LS_ANG_INERTIA)); UI::SetNextItemWidth(300);
            float nAI = UI::SliderFloat("##ai", S_AngInertia, 0.0f, 0.90f);
            if (nAI != S_AngInertia) { S_AngInertia = nAI; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_CAM_TILT)); UI::SetNextItemWidth(300);
            float nCT = UI::SliderFloat("##ct", S_CameraTilt, -100.0f, 100.0f);
            if (nCT != S_CameraTilt) { S_CameraTilt = nCT; phyChanged = true; }

            bool nVR = UI::Checkbox(T(LS_VISUAL_ROLL), S_VisualRoll);
            if (nVR != S_VisualRoll) { S_VisualRoll = nVR; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_DRAG_H)); UI::SetNextItemWidth(300);
            float nDH = UI::SliderFloat("##dh", S_Drag, 0.0f, 0.50f);
            if (nDH != S_Drag) { S_Drag = nDH; phyChanged = true; }

            UI::Text(T(LS_DRAG_V)); UI::SetNextItemWidth(300);
            float nDV = UI::SliderFloat("##dv", S_DragY, 0.0f, 0.30f);
            if (nDV != S_DragY) { S_DragY = nDV; phyChanged = true; }

            UI::Text(T(LS_QDRAG_H)); UI::SetNextItemWidth(300);
            float nQDH = UI::SliderFloat("##qdh", S_QuadDrag, 0.0f, 0.20f);
            if (nQDH != S_QuadDrag) { S_QuadDrag = nQDH; phyChanged = true; }

            UI::Text(T(LS_QDRAG_V)); UI::SetNextItemWidth(300);
            float nQDV = UI::SliderFloat("##qdv", S_QuadDragY, 0.0f, 0.20f);
            if (nQDV != S_QuadDragY) { S_QuadDragY = nQDV; phyChanged = true; }

            UI::Separator();
            bool nGE = UI::Checkbox(T(LS_GRAVITY_EN), S_GravityEnabled);
            if (nGE != S_GravityEnabled) { S_GravityEnabled = nGE; phyChanged = true; }

            UI::Text(T(LS_GRAVITY)); UI::SetNextItemWidth(300);
            float nGrav = UI::SliderFloat("##grav", S_Gravity, 0.0f, 200.0f);
            if (nGrav != S_Gravity) { S_Gravity = nGrav; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_DEADZONE)); UI::SetNextItemWidth(300);
            float nDZ = UI::SliderFloat("##dz", S_Deadzone, 0.0f, 0.40f);
            if (nDZ != S_Deadzone) { S_Deadzone = nDZ; phyChanged = true; }

            bool nMH = UI::Checkbox(T(LS_MIN_H_EN), S_UseMinHeight);
            if (nMH != S_UseMinHeight) { S_UseMinHeight = nMH; phyChanged = true; }

            UI::Text(T(LS_MIN_H)); UI::SetNextItemWidth(300);
            float nMHv = UI::SliderFloat("##mh", S_MinHeight, 0.0f, 50.0f);
            if (nMHv != S_MinHeight) { S_MinHeight = nMHv; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_THR_MAX)); UI::SetNextItemWidth(300);
            float nTM = UI::SliderFloat("##tm", S_ThrottleMax, 1.0f, 2.0f);
            if (nTM != S_ThrottleMax) { S_ThrottleMax = nTM; phyChanged = true; }

            UI::Text(T(LS_THR_CURVE)); UI::SetNextItemWidth(300);
            float nTC = UI::SliderFloat("##tc", S_ThrottleCurve, 0.5f, 3.0f);
            if (nTC != S_ThrottleCurve) { S_ThrottleCurve = nTC; phyChanged = true; }

            UI::Text(T(LS_MIN_THR_PCT)); UI::SetNextItemWidth(300);
            float nMTP = UI::SliderFloat("##mtp", S_MinThrottlePct, 0.0f, 30.0f);
            if (nMTP != S_MinThrottlePct) { S_MinThrottlePct = nMTP; phyChanged = true; }

            UI::Separator();
            UI::Text(T(LS_MOTOR_TAU)); UI::SetNextItemWidth(300);
            float nTau = UI::SliderFloat("##tau", S_MotorTau, 0.0f, 0.15f);
            if (nTau != S_MotorTau) { S_MotorTau = nTau; phyChanged = true; }

            UI::Text(T(LS_GYRO)); UI::SetNextItemWidth(300);
            float nGyro = UI::SliderFloat("##gyro", S_GyroCoeff, 0.0f, 5.0f);
            if (nGyro != S_GyroCoeff) { S_GyroCoeff = nGyro; phyChanged = true; }

            UI::Separator();
            if (phyChanged) SavePhysics();

            UI::Text("");
            UI::Text(T(LS_PRESET_SECT));
            if (UI::Button(T(LS_UNCRASHED_BTN)))
                ApplyUncrashed5FreestylePreset();
            if (g_UncrashedPresetStatus.Length > 0) {
                UI::Text(g_UncrashedPresetStatus);
                if (g_UncrashedTestReport.Length > 0) UI::Text(g_UncrashedTestReport);
            }
            UI::EndTabItem();
        }

        // ── Onglet Axes ──────────────────────────────────────────
        if (UI::BeginTabItem(T(LS_AXES_TAB))) {
            UI::Text("");
            bool axesChanged = false;

            bool nIT = UI::Checkbox(T(LS_INV_THROTTLE), S_InvThrottle);
            if (nIT != S_InvThrottle) { S_InvThrottle = nIT; axesChanged = true; }

            bool nIY = UI::Checkbox(T(LS_INV_YAW), S_InvYaw);
            if (nIY != S_InvYaw) { S_InvYaw = nIY; axesChanged = true; }

            bool nIP = UI::Checkbox(T(LS_INV_PITCH), S_InvPitch);
            if (nIP != S_InvPitch) { S_InvPitch = nIP; axesChanged = true; }

            bool nIR = UI::Checkbox(T(LS_INV_ROLL), S_InvRoll);
            if (nIR != S_InvRoll) { S_InvRoll = nIR; axesChanged = true; }

            bool nIVR = UI::Checkbox(T(LS_INV_VIS_ROLL), S_InvVisualRoll);
            if (nIVR != S_InvVisualRoll) { S_InvVisualRoll = nIVR; axesChanged = true; }

            if (axesChanged) SavePhysics();
            UI::EndTabItem();
        }

        // ── Onglet Binds ─────────────────────────────────────────
        if (UI::BeginTabItem(T(LS_BINDS_TAB))) {
            UI::Text(T(LS_BIND_HINT1));
            UI::Text(T(LS_BIND_HINT2));
            UI::Text("");

            // Détection de la touche en cours de capture
            if (g_CapturingBind >= 0) {
                if (g_CapturingBind == 0 || g_CapturingBind == 2 || g_CapturingBind == 4 || g_CapturingBind == 6) {
                    auto padC = GetGamepad();
                    if (padC !is null) {
                        for (int bi = 1; bi < int(PAD_LABELS.Length); bi++) {
                            if (IsPadBtnDown(padC, bi)) {
                                if      (g_CapturingBind == 0) S_RecPadBtn     = bi;
                                else if (g_CapturingBind == 2) S_RespawnPadBtn = bi;
                                else if (g_CapturingBind == 4) S_CpPadBtn      = bi;
                                else                           S_RewindPadBtn  = bi;
                                g_CapturingBind = -1;
                                SaveBinds();
                                break;
                            }
                        }
                    }
                } else {
                    for (int ki = 1; ki < int(KEY_LABELS.Length); ki++) {
                        if (IsKeyBtnDown(ki)) {
                            if      (g_CapturingBind == 1) S_RecKey     = ki;
                            else if (g_CapturingBind == 3) S_RespawnKey = ki;
                            else if (g_CapturingBind == 5) S_CpKey      = ki;
                            else                           S_RewindKey  = ki;
                            g_CapturingBind = -1;
                            SaveBinds();
                            break;
                        }
                    }
                }
                if (UI::IsKeyPressed(UI::Key::Escape)) g_CapturingBind = -1;
            }

            UI::Separator();
            UI::Text(T(LS_RECORD));
            UI::SameLine(120); UI::Text(T(LS_PAD_LBL)); UI::SameLine(170);
            S_RecPadBtn = BindBtn("rp", PAD_LABELS, S_RecPadBtn, 0);
            UI::SameLine(350); UI::Text(T(LS_KEY_LBL)); UI::SameLine(420);
            S_RecKey    = BindBtn("rk", KEY_LABELS, S_RecKey,    1);

            UI::Separator();
            UI::Text(T(LS_RESPAWN));
            UI::SameLine(120); UI::Text(T(LS_PAD_LBL)); UI::SameLine(170);
            S_RespawnPadBtn = BindBtn("sp", PAD_LABELS, S_RespawnPadBtn, 2);
            UI::SameLine(350); UI::Text(T(LS_KEY_LBL)); UI::SameLine(420);
            S_RespawnKey    = BindBtn("sk", KEY_LABELS, S_RespawnKey,    3);

            UI::Separator();
            UI::Text(T(LS_CHECKPOINT));
            UI::SameLine(120); UI::Text(T(LS_PAD_LBL)); UI::SameLine(170);
            S_CpPadBtn = BindBtn("cp", PAD_LABELS, S_CpPadBtn, 4);
            UI::SameLine(350); UI::Text(T(LS_KEY_LBL)); UI::SameLine(420);
            S_CpKey    = BindBtn("ck", KEY_LABELS, S_CpKey,    5);

            UI::Separator();
            UI::Text(T(LS_REWIND));
            UI::SameLine(120); UI::Text(T(LS_PAD_LBL)); UI::SameLine(170);
            S_RewindPadBtn = BindBtn("rwp", PAD_LABELS, S_RewindPadBtn, 6);
            UI::SameLine(350); UI::Text(T(LS_KEY_LBL)); UI::SameLine(420);
            S_RewindKey    = BindBtn("rwk", KEY_LABELS, S_RewindKey,    7);

            UI::Separator();
            UI::Text(T(LS_CP_HINT));
            UI::EndTabItem();
        }

        // ── Onglet Misc ──────────────────────────────────────────
        if (UI::BeginTabItem(T(LS_MISC_TAB))) {

            // Sélecteur de langue
            UI::Text(T(LS_LANG_LBL));
            UI::SameLine(130);
            for (int li = 0; li < int(LANG_NAMES.Length); li++) {
                if (li > 0) UI::SameLine();
                bool sel = (g_Lang == li);
                if (sel) UI::PushStyleColor(UI::Col::Button, vec4(0.15f, 0.45f, 0.85f, 1.0f));
                if (UI::Button(LANG_NAMES[li] + "##lang" + li)) {
                    if (g_Lang != li) { g_Lang = li; InitLoc(); SaveMisc(); }
                }
                if (sel) UI::PopStyleColor(1);
            }

            UI::Text("");
            UI::Text(T(LS_OVERLAY_SECT));
            UI::Text("");

            bool changed = false;

            bool nSho = UI::Checkbox(T(LS_SHOW_OVERLAY), S_ShowInputOverlay);
            if (nSho != S_ShowInputOverlay) { S_ShowInputOverlay = nSho; changed = true; }

            bool nVal = UI::Checkbox(T(LS_SHOW_VALUES), g_ShowLiveValues);
            if (nVal != g_ShowLiveValues) { g_ShowLiveValues = nVal; changed = true; }


            UI::Text("");

            UI::Text(T(LS_KF_FREQ)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            int nKf = UI::SliderInt("##kf", S_KfFreq, 1, 60);
            if (nKf != S_KfFreq) { S_KfFreq = nKf; changed = true; }

            UI::Text("");
            UI::Text(T(LS_POS_X)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            int newX = UI::SliderInt("##ox", S_OverlayX, 0, 3840);
            if (newX != S_OverlayX) { S_OverlayX = newX; changed = true; }

            UI::Text(T(LS_POS_Y)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            int newY = UI::SliderInt("##oy", S_OverlayY, 0, 2160);
            if (newY != S_OverlayY) { S_OverlayY = newY; changed = true; }

            UI::Text(T(LS_SZ)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nSz = UI::SliderFloat("##sz", g_OvSz, 40.0f, 150.0f);
            if (nSz != g_OvSz) { g_OvSz = nSz; changed = true; }

            UI::Text(T(LS_GAP)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nGap = UI::SliderFloat("##gap", g_OvGap, 0.0f, 60.0f);
            if (nGap != g_OvGap) { g_OvGap = nGap; changed = true; }

            UI::Text(T(LS_BG_ALPHA)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nBg = UI::SliderFloat("##bg", g_OvBgAlpha, 0.0f, 1.0f);
            if (nBg != g_OvBgAlpha) { g_OvBgAlpha = nBg; changed = true; }

            UI::Text(T(LS_BORDER_ALPHA)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nBr = UI::SliderFloat("##br", g_OvBorderAlpha, 0.0f, 1.0f);
            if (nBr != g_OvBorderAlpha) { g_OvBorderAlpha = nBr; changed = true; }

            UI::Text(T(LS_CROSS_ALPHA)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nCr = UI::SliderFloat("##cr", g_OvCrossAlpha, 0.0f, 1.0f);
            if (nCr != g_OvCrossAlpha) { g_OvCrossAlpha = nCr; changed = true; }

            UI::Text(T(LS_DOT_SIZE)); UI::SameLine(160);
            UI::SetNextItemWidth(200);
            float nDs = UI::SliderFloat("##ds", g_OvDotSize, 1.5f, 14.0f);
            if (nDs != g_OvDotSize) { g_OvDotSize = nDs; changed = true; }

            UI::Text(T(LS_DOT_COLOR)); UI::SameLine(160);
            UI::SetNextItemWidth(60); float nR = UI::SliderFloat("R##dr", g_OvDotR, 0.0f, 1.0f);
            UI::SameLine(); UI::SetNextItemWidth(60); float nG = UI::SliderFloat("G##dg", g_OvDotG, 0.0f, 1.0f);
            UI::SameLine(); UI::SetNextItemWidth(60); float nB = UI::SliderFloat("B##db", g_OvDotB, 0.0f, 1.0f);
            if (nR != g_OvDotR || nG != g_OvDotG || nB != g_OvDotB) {
                g_OvDotR = nR; g_OvDotG = nG; g_OvDotB = nB; changed = true;
            }

            if (changed) SaveMisc();

            UI::EndTabItem();
        }

        // ── Onglet Aide / Help ───────────────────────────────────
        if (UI::BeginTabItem(T(LS_HELP_TAB))) {
            UI::Text("\\$888FPV Drone ─────────────────────────────────────");
            UI::Text("");
            UI::TextWrapped(T(LS_HELP_WHAT));
            UI::Text("");
            UI::TextWrapped(T(LS_HELP_REQ));
            UI::Text("");
            UI::TextWrapped(T(LS_HELP_CTRL));
            UI::Text("");
            UI::TextWrapped(T(LS_HELP_REC));
            UI::Text("");
            UI::TextWrapped(T(LS_HELP_CP));
            UI::Text("");
UI::TextWrapped(T(LS_HELP_PRESETS));
            UI::Text("");
            UI::Text(T(LS_EXPORT_SECT));
            UI::TextWrapped(T(LS_EXPORT_STEP1));
            auto storagePath = IO::FromStorageFolder("");
            UI::TextWrapped(T(LS_EXPORT_STEP2));
            if (UI::Button(T(LS_EXPORT_BTN) + "##export")) {
                OpenExplorerPath(storagePath);
            }
            UI::Text("");
            UI::TextWrapped(T(LS_EXPORT_STEP3));
            UI::SameLine();
            UI::TextLinkOpenURL("ici", "https://utils.tmtas.exchange/fpvclip.html");
            UI::SameLine();
            UI::Text(T(LS_EXPORT_LINK));
            UI::EndTabItem();
        }

        UI::EndTabBar();
    }
}

// ── MISC PERSISTENCE ─────────────────────────────────────────

string MiscPath() { return IO::FromStorageFolder("misc.txt"); }

void SaveMisc() {
    IO::File f(MiscPath(), IO::FileMode::Write);
    f.WriteLine("ShowOverlay=" + (S_ShowInputOverlay ? 1 : 0));
    f.WriteLine("ShowValues="  + (g_ShowLiveValues   ? 1 : 0));
    f.WriteLine("OverlayX="   + S_OverlayX);
    f.WriteLine("OverlayY="   + S_OverlayY);
    f.WriteLine("Sz="          + g_OvSz);
    f.WriteLine("Gap="         + g_OvGap);
    f.WriteLine("BgAlpha="     + g_OvBgAlpha);
    f.WriteLine("BorderAlpha=" + g_OvBorderAlpha);
    f.WriteLine("CrossAlpha="  + g_OvCrossAlpha);
    f.WriteLine("DotSize="     + g_OvDotSize);
    f.WriteLine("DotR="        + g_OvDotR);
    f.WriteLine("DotG="        + g_OvDotG);
    f.WriteLine("DotB="        + g_OvDotB);
    f.WriteLine("Lang="        + g_Lang);
    f.Close();
}

void LoadMisc() {
    string path = MiscPath();
    if (!IO::FileExists(path)) return;
    IO::File f(path, IO::FileMode::Read);
    string content = f.ReadToEnd();
    f.Close();
    int start = 0;
    while (start < int(content.Length)) {
        int end = FindNL(content, start);
        if (end < 0) end = int(content.Length);
        string line = content.SubStr(start, end - start).Trim();
        int eq = -1;
        for (uint ci = 0; ci < line.Length; ci++) { if (line[ci] == 61) { eq = int(ci); break; } }
        if (eq > 0) {
            string k = line.SubStr(0, eq).Trim();
            float  v = Text::ParseFloat(line.SubStr(eq + 1).Trim());
            if      (k == "ShowOverlay")  S_ShowInputOverlay = (v > 0.5f);
            else if (k == "ShowValues")   g_ShowLiveValues   = (v > 0.5f);
            else if (k == "OverlayX")    S_OverlayX      = int(v);
            else if (k == "OverlayY")    S_OverlayY      = int(v);
            else if (k == "Sz")          g_OvSz          = v;
            else if (k == "Gap")         g_OvGap         = v;
            else if (k == "BgAlpha")     g_OvBgAlpha     = v;
            else if (k == "BorderAlpha") g_OvBorderAlpha = v;
            else if (k == "CrossAlpha")  g_OvCrossAlpha  = v;
            else if (k == "DotSize")     g_OvDotSize     = v;
            else if (k == "DotR")        g_OvDotR        = v;
            else if (k == "DotG")        g_OvDotG        = v;
            else if (k == "DotB")        g_OvDotB        = v;
            else if (k == "Lang")        g_Lang          = Math::Clamp(int(v), 0, int(LANG_NAMES.Length) - 1);
        }
        if (end >= int(content.Length)) break;
        start = end + 1;
    }
}

// ── PHYSICS PERSISTENCE ──────────────────────────────────────

string PhysicsPath() { return IO::FromStorageFolder("physics.txt"); }

void SavePhysics() {
    IO::File f(PhysicsPath(), IO::FileMode::Write);
    f.WriteLine("Enabled="        + (S_Enabled        ? 1 : 0));
    f.WriteLine("Thrust="         + S_Thrust);
    f.WriteLine("TiltRate="       + S_TiltRate);
    f.WriteLine("YawRate="        + S_YawRate);
    f.WriteLine("AngInertia="     + S_AngInertia);
    f.WriteLine("CameraTilt="     + S_CameraTilt);
    f.WriteLine("VisualRoll="     + (S_VisualRoll     ? 1 : 0));
    f.WriteLine("Drag="           + S_Drag);
    f.WriteLine("DragY="          + S_DragY);
    f.WriteLine("QuadDrag="       + S_QuadDrag);
    f.WriteLine("QuadDragY="      + S_QuadDragY);
    f.WriteLine("GravityEnabled=" + (S_GravityEnabled ? 1 : 0));
    f.WriteLine("Gravity="        + S_Gravity);
    f.WriteLine("Deadzone="       + S_Deadzone);
    f.WriteLine("UseMinHeight="   + (S_UseMinHeight   ? 1 : 0));
    f.WriteLine("MinHeight="      + S_MinHeight);
    f.WriteLine("ThrottleMax="    + S_ThrottleMax);
    f.WriteLine("ThrottleCurve="  + S_ThrottleCurve);
    f.WriteLine("MinThrottlePct=" + S_MinThrottlePct);
    f.WriteLine("MotorTau="       + S_MotorTau);
    f.WriteLine("GyroCoeff="      + S_GyroCoeff);
    f.WriteLine("InvThrottle="    + (S_InvThrottle    ? 1 : 0));
    f.WriteLine("InvYaw="         + (S_InvYaw         ? 1 : 0));
    f.WriteLine("InvPitch="       + (S_InvPitch       ? 1 : 0));
    f.WriteLine("InvRoll="        + (S_InvRoll        ? 1 : 0));
    f.WriteLine("InvVisualRoll="  + (S_InvVisualRoll  ? 1 : 0));
    f.WriteLine("KfFreq="        + S_KfFreq);
    f.Close();
}

void LoadPhysics() {
    string path = PhysicsPath();
    if (!IO::FileExists(path)) return;
    IO::File f(path, IO::FileMode::Read);
    string content = f.ReadToEnd();
    f.Close();
    int start = 0;
    while (start < int(content.Length)) {
        int end = FindNL(content, start);
        if (end < 0) end = int(content.Length);
        string line = content.SubStr(start, end - start).Trim();
        int eq = -1;
        for (uint ci = 0; ci < line.Length; ci++) { if (line[ci] == 61) { eq = int(ci); break; } }
        if (eq > 0) {
            string k = line.SubStr(0, eq).Trim();
            float  v = Text::ParseFloat(line.SubStr(eq + 1).Trim());
            if      (k == "Enabled")        S_Enabled        = (v > 0.5f);
            else if (k == "Thrust")         S_Thrust         = v;
            else if (k == "TiltRate")       S_TiltRate       = v;
            else if (k == "YawRate")        S_YawRate        = v;
            else if (k == "AngInertia")     S_AngInertia     = v;
            else if (k == "CameraTilt")     S_CameraTilt     = v;
            else if (k == "VisualRoll")     S_VisualRoll     = (v > 0.5f);
            else if (k == "Drag")           S_Drag           = v;
            else if (k == "DragY")          S_DragY          = v;
            else if (k == "QuadDrag")       S_QuadDrag       = v;
            else if (k == "QuadDragY")      S_QuadDragY      = v;
            else if (k == "GravityEnabled") S_GravityEnabled = (v > 0.5f);
            else if (k == "Gravity")        S_Gravity        = v;
            else if (k == "Deadzone")       S_Deadzone       = v;
            else if (k == "UseMinHeight")   S_UseMinHeight   = (v > 0.5f);
            else if (k == "MinHeight")      S_MinHeight      = v;
            else if (k == "ThrottleMax")    S_ThrottleMax    = v;
            else if (k == "ThrottleCurve")  S_ThrottleCurve  = v;
            else if (k == "MinThrottlePct") S_MinThrottlePct = v;
            else if (k == "MotorTau")       S_MotorTau       = v;
            else if (k == "GyroCoeff")      S_GyroCoeff      = v;
            else if (k == "InvThrottle")    S_InvThrottle    = (v > 0.5f);
            else if (k == "InvYaw")         S_InvYaw         = (v > 0.5f);
            else if (k == "InvPitch")       S_InvPitch       = (v > 0.5f);
            else if (k == "InvRoll")        S_InvRoll        = (v > 0.5f);
            else if (k == "InvVisualRoll")  S_InvVisualRoll  = (v > 0.5f);
            else if (k == "KfFreq")        S_KfFreq        = v;
        }
        if (end >= int(content.Length)) break;
        start = end + 1;
    }
}

// ── BINDS PERSISTENCE ────────────────────────────────────────

string BindsPath() { return IO::FromStorageFolder("binds.txt"); }

void SaveBinds() {
    IO::File f(BindsPath(), IO::FileMode::Write);
    f.WriteLine("RecPad="     + S_RecPadBtn);
    f.WriteLine("RecKey="     + S_RecKey);
    f.WriteLine("RespawnPad=" + S_RespawnPadBtn);
    f.WriteLine("RespawnKey=" + S_RespawnKey);
    f.WriteLine("CpPad="      + S_CpPadBtn);
    f.WriteLine("CpKey="      + S_CpKey);
    f.WriteLine("RewindPad="  + S_RewindPadBtn);
    f.WriteLine("RewindKey="  + S_RewindKey);
    f.Close();
}

void LoadBinds() {
    string path = BindsPath();
    if (!IO::FileExists(path)) return;
    IO::File f(path, IO::FileMode::Read);
    string content = f.ReadToEnd();
    f.Close();
    int start = 0;
    while (start < int(content.Length)) {
        int end = FindNL(content, start);
        if (end < 0) end = int(content.Length);
        string line = content.SubStr(start, end - start).Trim();
        int eq = -1;
        for (uint ci = 0; ci < line.Length; ci++) { if (line[ci] == 61) { eq = int(ci); break; } }
        if (eq > 0) {
            string k = line.SubStr(0, eq).Trim();
            int    v = Text::ParseInt(line.SubStr(eq + 1).Trim());
            if      (k == "RecPad")     S_RecPadBtn     = v;
            else if (k == "RecKey")     S_RecKey        = v;
            else if (k == "RespawnPad") S_RespawnPadBtn = v;
            else if (k == "RespawnKey") S_RespawnKey    = v;
            else if (k == "CpPad")      S_CpPadBtn      = v;
            else if (k == "CpKey")      S_CpKey         = v;
            else if (k == "RewindPad")  S_RewindPadBtn  = v;
            else if (k == "RewindKey")  S_RewindKey     = v;
        }
        if (end >= int(content.Length)) break;
        start = end + 1;
    }
}

// ── BIND HELPERS ─────────────────────────────────────────────

bool IsPadBtnDown(CInputScriptPad@ pad, int idx) {
    switch (idx) {
        case  1: return pad.A > 0;
        case  2: return pad.B > 0;
        case  3: return pad.X > 0;
        case  4: return pad.Y > 0;
        case  5: return pad.L1 > 0;
        case  6: return pad.R1 > 0;
        case  7: return pad.L2 >= 0.5f;
        case  8: return pad.R2 >= 0.5f;
        case  9: return pad.LeftStickBut > 0;
        case 10: return pad.RightStickBut > 0;
        case 11: return pad.Up > 0;
        case 12: return pad.Down > 0;
        case 13: return pad.Left > 0;
        case 14: return pad.Right > 0;
        case 15: return pad.Menu > 0;
        case 16: return pad.View > 0;
    }
    return false;
}

bool IsKeyBtnDown(int idx) {
    switch (idx) {
        case  1: return UI::IsKeyPressed(UI::Key::F1);
        case  2: return UI::IsKeyPressed(UI::Key::F2);
        case  3: return UI::IsKeyPressed(UI::Key::F3);
        case  4: return UI::IsKeyPressed(UI::Key::F4);
        case  5: return UI::IsKeyPressed(UI::Key::F5);
        case  6: return UI::IsKeyPressed(UI::Key::F6);
        case  7: return UI::IsKeyPressed(UI::Key::F7);
        case  8: return UI::IsKeyPressed(UI::Key::F8);
        case  9: return UI::IsKeyPressed(UI::Key::F9);
        case 10: return UI::IsKeyPressed(UI::Key::F10);
        case 11: return UI::IsKeyPressed(UI::Key::F11);
        case 12: return UI::IsKeyPressed(UI::Key::F12);
    }
    return false;
}

bool IsKeyBtnHeld(int idx) {
    switch (idx) {
        case  1: return UI::IsKeyDown(UI::Key::F1);
        case  2: return UI::IsKeyDown(UI::Key::F2);
        case  3: return UI::IsKeyDown(UI::Key::F3);
        case  4: return UI::IsKeyDown(UI::Key::F4);
        case  5: return UI::IsKeyDown(UI::Key::F5);
        case  6: return UI::IsKeyDown(UI::Key::F6);
        case  7: return UI::IsKeyDown(UI::Key::F7);
        case  8: return UI::IsKeyDown(UI::Key::F8);
        case  9: return UI::IsKeyDown(UI::Key::F9);
        case 10: return UI::IsKeyDown(UI::Key::F10);
        case 11: return UI::IsKeyDown(UI::Key::F11);
        case 12: return UI::IsKeyDown(UI::Key::F12);
    }
    return false;
}

// ── PRESETS UTILISATEUR ──────────────────────────────────────

int FindNL(const string &in s, int from) {
    uint start = from < 0 ? 0 : uint(from);
    for (uint i = start; i < s.Length; i++) {
        if (s[i] == 10) return int(i);
    }
    return -1;
}

string PresetPath(int slot) {
    return IO::FromStorageFolder("preset" + (slot + 1) + ".txt");
}

void SavePreset(int slot) {
    string c = "";
    c += "Name="           + g_PresetNames[slot]          + "\n";
    c += "Thrust="         + S_Thrust                      + "\n";
    c += "TiltRate="       + S_TiltRate                    + "\n";
    c += "YawRate="        + S_YawRate                     + "\n";
    c += "AngInertia="     + S_AngInertia                  + "\n";
    c += "CameraTilt="     + S_CameraTilt                  + "\n";
    c += "VisualRoll="     + (S_VisualRoll ? 1 : 0)        + "\n";
    c += "Drag="           + S_Drag                        + "\n";
    c += "DragY="          + S_DragY                       + "\n";
    c += "QuadDrag="       + S_QuadDrag                    + "\n";
    c += "QuadDragY="      + S_QuadDragY                   + "\n";
    c += "GravityEnabled=" + (S_GravityEnabled ? 1 : 0)    + "\n";
    c += "Gravity="        + S_Gravity                     + "\n";
    c += "Deadzone="       + S_Deadzone                    + "\n";
    c += "UseMinHeight="   + (S_UseMinHeight ? 1 : 0)      + "\n";
    c += "MinHeight="      + S_MinHeight                   + "\n";
    c += "ThrottleMax="    + S_ThrottleMax                 + "\n";
    c += "ThrottleCurve="  + S_ThrottleCurve               + "\n";
    c += "MinThrottlePct=" + S_MinThrottlePct              + "\n";
    c += "MotorTau="       + S_MotorTau                    + "\n";
    c += "GyroCoeff="      + S_GyroCoeff                   + "\n";
    c += "InvThrottle="    + (S_InvThrottle ? 1 : 0)       + "\n";
    c += "InvYaw="         + (S_InvYaw ? 1 : 0)            + "\n";
    c += "InvPitch="       + (S_InvPitch ? 1 : 0)          + "\n";
    c += "InvRoll="        + (S_InvRoll ? 1 : 0)           + "\n";
    c += "InvVisualRoll="  + (S_InvVisualRoll ? 1 : 0)     + "\n";

    IO::File f(PresetPath(slot), IO::FileMode::Write);
    f.Write(c);
    f.Close();
    g_PresetStatus = "Preset " + (slot + 1) + " \"" + g_PresetNames[slot] + "\" sauvegardé.";
    g_PresetError  = false;
}

void ApplyPresetKV(int slot, const string &in key, const string &in val) {
    if (key == "Name") { g_PresetNames[slot] = val; return; }
    float fv = Text::ParseFloat(val);
    bool  bv = (val == "1");
    if      (key == "Thrust")         S_Thrust         = fv;
    else if (key == "TiltRate")       S_TiltRate       = fv;
    else if (key == "YawRate")        S_YawRate        = fv;
    else if (key == "AngInertia")     S_AngInertia     = fv;
    else if (key == "CameraTilt")     S_CameraTilt     = fv;
    else if (key == "VisualRoll")     S_VisualRoll     = bv;
    else if (key == "Drag")           S_Drag           = fv;
    else if (key == "DragY")          S_DragY          = fv;
    else if (key == "QuadDrag")       S_QuadDrag       = fv;
    else if (key == "QuadDragY")      S_QuadDragY      = fv;
    else if (key == "GravityEnabled") S_GravityEnabled = bv;
    else if (key == "Gravity")        S_Gravity        = fv;
    else if (key == "Deadzone")       S_Deadzone       = fv;
    else if (key == "UseMinHeight")   S_UseMinHeight   = bv;
    else if (key == "MinHeight")      S_MinHeight      = fv;
    else if (key == "ThrottleMax")    S_ThrottleMax    = fv;
    else if (key == "ThrottleCurve")  S_ThrottleCurve  = fv;
    else if (key == "MinThrottlePct") S_MinThrottlePct = fv;
    else if (key == "MotorTau")       S_MotorTau       = fv;
    else if (key == "GyroCoeff")      S_GyroCoeff      = fv;
    else if (key == "InvThrottle")    S_InvThrottle    = bv;
    else if (key == "InvYaw")         S_InvYaw         = bv;
    else if (key == "InvPitch")       S_InvPitch       = bv;
    else if (key == "InvRoll")        S_InvRoll        = bv;
    else if (key == "InvVisualRoll")  S_InvVisualRoll  = bv;
}

void LoadPreset(int slot) {
    string path = PresetPath(slot);
    if (!IO::FileExists(path)) {
        g_PresetStatus = "Aucun preset " + (slot + 1) + " trouvé.";
        g_PresetError  = true;
        return;
    }
    IO::File f(path, IO::FileMode::Read);
    string content = f.ReadToEnd();
    f.Close();

    int start = 0;
    while (start <= int(content.Length)) {
        int end = FindNL(content, start);
        if (end < 0) end = int(content.Length);
        string line = content.SubStr(start, end - start).Trim();
        if (line.Length > 0) {
            int eq = -1;
            for (uint ci = 0; ci < line.Length; ci++) {
                if (line[ci] == 61) { eq = int(ci); break; }  // '=' = 61
            }
            if (eq > 0)
                ApplyPresetKV(slot, line.SubStr(0, eq).Trim(), line.SubStr(eq + 1).Trim());
        }
        if (end >= int(content.Length)) break;
        start = end + 1;
    }
    g_PresetStatus = "Preset " + (slot + 1) + " \"" + g_PresetNames[slot] + "\" chargé.";
    g_PresetError  = false;
}

// Lit uniquement le nom depuis un fichier preset (1ère ligne Name=...)
string ReadPresetName(int slot) {
    string path = PresetPath(slot);
    if (!IO::FileExists(path)) return "Preset " + (slot + 1);
    IO::File f(path, IO::FileMode::Read);
    string content = f.ReadToEnd();
    f.Close();
    int nl = FindNL(content, 0);
    if (nl < 0) nl = int(content.Length);
    string line = content.SubStr(0, nl).Trim();
    int eq = -1;
    for (uint ci = 0; ci < line.Length; ci++) {
        if (line[ci] == 61) { eq = int(ci); break; }
    }
    if (eq > 0 && line.SubStr(0, eq).Trim() == "Name")
        return line.SubStr(eq + 1).Trim();
    return "Preset " + (slot + 1);
}

// ── MATH 3D ───────────────────────────────────────────────────

float VDot(vec3 a, vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

vec3 VCross(vec3 a, vec3 b) {
    return vec3(a.y*b.z - a.z*b.y,
                a.z*b.x - a.x*b.z,
                a.x*b.y - a.y*b.x);
}

vec3 VNorm(vec3 v) {
    float len = Math::Sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.00001f) return vec3(0, 1, 0);
    return vec3(v.x/len, v.y/len, v.z/len);
}

vec3 RotAround(vec3 v, vec3 axis, float angle) {
    float c = Math::Cos(angle), s = Math::Sin(angle);
    float d = VDot(axis, v);
    vec3  cv = VCross(axis, v);
    return vec3(v.x*c + cv.x*s + axis.x*d*(1.0f-c),
                v.y*c + cv.y*s + axis.y*d*(1.0f-c),
                v.z*c + cv.z*s + axis.z*d*(1.0f-c));
}

// ── ACCÈS CAMÉRA LIBRE ────────────────────────────────────────

uint16 GetMemberOffset(const string &in cls, const string &in mem) {
    return Reflection::GetType(cls).GetMember(mem).Offset;
}

CGameControlCameraFree@ GetFreeCam() {
    auto app = GetApp();
    if (app is null || app.GameScene is null) return null;
    uint16 off = GetMemberOffset("CGameManiaPlanet", "GameScene");
    auto mgr   = Dev::GetOffsetNod(app, off + 0x10);
    if (mgr is null) return null;
    if ((Dev::GetOffsetUint64(mgr, 0x80) & 0xF) != 0) return null;
    return cast<CGameControlCameraFree>(Dev::GetOffsetNod(mgr, 0x80));
}

// ── ACCÈS MANETTE ─────────────────────────────────────────────

CInputScriptPad@ GetGamepad() {
    auto port = GetApp().InputPort;
    if (port is null) return null;
    for (uint i = 0; i < port.Script_Pads.Length; i++) {
        auto p = port.Script_Pads[i];
        if (p is null) continue;
        if (p.Type == CInputScriptPad::EPadType::Keyboard) continue;
        if (p.Type == CInputScriptPad::EPadType::Mouse)    continue;
        return p;
    }
    return null;
}

float DZ(float v) { return (Math::Abs(v) < S_Deadzone) ? 0.0f : v; }

CGameEditorMediaTracker@ GetMtEditor() {
    auto app = GetApp();
    if (app is null) return null;
    auto mt = cast<CGameEditorMediaTracker>(app.Editor);
    if (mt is null) @mt = cast<CGameEditorMediaTracker>(app.EditorBase);
    return mt;
}

CGameEditorMediaTrackerPluginAPI@ GetMtPlugin() {
    auto mtEd = GetMtEditor();
    if (mtEd is null) return null;
    return cast<CGameEditorMediaTrackerPluginAPI>(mtEd.PluginAPI);
}

// ── EXPORT CSV ───────────────────────────────────────────────
// Format : time,x,y,z,pitch,yaw,roll  (une ligne par keyframe décimé)
// Fichier : PluginStorage/FPVDrone/trajectory.csv

string g_ExportStatus = "";
bool   g_ExportError  = false;

void ExportToCSV() {
    if (g_TrkT.Length < 2) {
        g_ExportStatus = "ERREUR: pas de trajectoire enregistrée";
        g_ExportError  = true;
        return;
    }

    int total = int(g_TrkT.Length);

    string csv = "";
    for (int i = 0; i < total; i++) {
        csv += Text::Format("%.6f", g_TrkT[i])     + ","
             + Text::Format("%.4f", g_TrkPos[i].x) + ","
             + Text::Format("%.4f", g_TrkPos[i].y) + ","
             + Text::Format("%.4f", g_TrkPos[i].z) + ","
             + Text::Format("%.6f", g_TrkPitch[i]) + ","
             + Text::Format("%.6f", g_TrkYaw[i])   + ","
             + Text::Format("%.6f", g_TrkRoll[i])  + "\n";
    }

    IO::File f(IO::FromStorageFolder("trajectory.csv"), IO::FileMode::Write);
    f.Write(csv);
    f.Close();

    g_ExportStatus = "CSV exporté — " + total + " lignes → PluginStorage/FPVDrone/trajectory.csv";
    g_ExportError  = false;
}

// ── INJECTION MEDIATRACKER ────────────────────────────────────

void InjectIntoMT() {
    g_Injecting    = true;
    g_InjectError  = false;
    g_InjectStatus = "démarrage...";

    if (g_TrkT.Length < 2) {
        g_InjectStatus = "ERREUR: pas de trajectoire enregistrée";
        g_InjectError = true;
        g_Injecting = false;
        return;
    }

    auto mt = GetMtPlugin();
    if (mt is null) {
        g_InjectStatus = "ERREUR: Plugin MT null — ouvre l'éditeur MediaTracker";
        g_InjectError = true;
        g_Injecting = false;
        return;
    }

    g_InjectStatus = "MT OK — création track...";
    yield();

    mt.CreateTrack(CGameEditorMediaTrackerPluginAPI::EMediaTrackerBlockType::CameraCustom);
    yield();

    uint trackIdx = mt.GetSelectedTrack();
    mt.SetTrackName(trackIdx, "FPV Drone");
    mt.RemoveAllKeys(trackIdx, 0);
    yield();

    int total = int(g_TrkT.Length);
    int maxKf = total;

    // Phase 1 : crée les keyframes (timing correct, position = celle du viewport)
    int created = 0;
    for (int i = 0; i < total; i++) {
        mt.CurrentTimer = g_TrkT[i];
        yield();
        mt.CreateKey();
        yield();
        created++;
        if (created % 10 == 0)
            g_InjectStatus = "kf... " + created + "/" + maxKf;
    }
    mt.CurrentTimer = 0.0f;
    yield();

    // Phase 2 : trouve le layout mémoire du block et écrase les positions
    auto clip = mt.Clip;
    if (clip is null || clip.Tracks.Length <= trackIdx) {
        g_InjectStatus = "OK " + created + " kf (clip inaccessible — positions non corrigées)";
        g_Injecting = false; return;
    }
    auto block = cast<CGameCtnMediaBlockCameraCustom>(clip.Tracks[trackIdx].Blocks[0]);
    if (block is null || block.GetKeysCount() != uint(created)) {
        g_InjectStatus = "OK " + created + " kf (block inaccessible — positions non corrigées)";
        g_Injecting = false; return;
    }

    // Cherche le temps du 2e keyframe dans la mémoire du block (plus distinctif que 0.0)
    int ki1 = (1 < total) ? 1 : 0;
    float t1 = g_TrkT[ki1];

    uint16 off1 = 0;
    for (uint16 off = 0x08; off < 0x3000; off += 4) {
        if (Math::Abs(Dev::GetOffsetFloat(block, off) - t1) < 0.0002f) {
            off1 = off; break;
        }
    }
    if (off1 == 0) {
        g_InjectStatus = "OK " + created + " kf — temps t1=" + t1 + " introuvable en mémoire";
        g_Injecting = false; return;
    }

    // Trouve le temps du 1er keyframe pour déduire le stride
    float t0 = g_TrkT[0];
    uint16 off0 = 0;
    for (uint16 off = 0x08; off < off1; off += 4) {
        if (Math::Abs(Dev::GetOffsetFloat(block, off) - t0) < 0.0002f) {
            off0 = off; break;
        }
    }
    if (off0 == 0) { off0 = off1; } // fallback si t0 == 0.0 ambigu

    uint16 stride = (ki1 > 0 && off1 > off0) ? uint16((off1 - off0) / ki1) : 64;

    // Écrase pos + angles dans chaque keyframe
    // Layout supposé : [time(+0), pos.x(+4), pos.y(+8), pos.z(+12), pitch(+16), yaw(+20), roll(+24)]
    for (int i = 0; i < total; i++) {
        uint16 base = uint16(off0 + i * stride);
        Dev::SetOffset(block, uint16(base +  4), g_TrkPos[i].x);
        Dev::SetOffset(block, uint16(base +  8), g_TrkPos[i].y);
        Dev::SetOffset(block, uint16(base + 12), g_TrkPos[i].z);
        Dev::SetOffset(block, uint16(base + 16), g_TrkPitch[i]);
        Dev::SetOffset(block, uint16(base + 20), g_TrkYaw[i]);
        Dev::SetOffset(block, uint16(base + 24), g_TrkRoll[i]);
    }

    g_InjectStatus = "OK — " + created + " kf injectés (stride=" + stride + " off0=0x" + Text::Format("%X", off0) + ")";
    g_Injecting = false;
}

// Neutralise l'input joystick de TM sur la caméra libre en mettant m_RotateSpeed=0
// via accès mémoire direct (la propriété est read-only dans l'API AngelScript).
// TM calcule la rotation caméra comme : delta = m_RotateSpeed × stickInput × dt
// → si m_RotateSpeed = 0, TM n'applique aucune rotation, quelle que soit le stick.
void SuppressTMCamInput(CGameControlCameraFree@ cam) {
    if (g_Off_RotSpd == 0) {
        g_Off_RotSpd = GetMemberOffset("CGameControlCameraFree", "m_RotateSpeed");
        g_Off_RotIn  = GetMemberOffset("CGameControlCameraFree", "m_RotateInertia");
    }
    Dev::SetOffset(cam, g_Off_RotSpd, 0.0f);
    Dev::SetOffset(cam, g_Off_RotIn,  0.0f);
}

// ── PHYSIQUE ACRO ─────────────────────────────────────────────

void ResetPhysics() {
    g_Fwd          = vec3(0, 0, 1);
    g_Up           = vec3(0, 1, 0);
    g_PitchRate    = 0; g_RollRate = 0; g_YawRateV = 0;
    g_Vel          = vec3(0, 0, 0);
    g_Throttle     = 0;
    g_MotorThrottle = 0;
    g_PosInit       = false;
    g_HasCheckpoint = false;
    g_CpRecTime      = 0.0f;
}

void SetCheckpoint() {
    g_CpPos        = g_Pos;  g_CpFwd = g_Fwd;  g_CpUp = g_Up;
    g_CpVel        = g_Vel;
    g_CpPitchRate  = g_PitchRate;
    g_CpRollRate   = g_CpRollRate;
    g_CpYawRate    = g_YawRateV;
    g_CpThrottle   = g_Throttle;
    g_CpMotorThr   = g_MotorThrottle;
    g_CpRecTime    = g_RecTime;
    g_HasCheckpoint = true;
}

void RestoreCheckpoint(CGameControlCameraFree@ cam) {
    g_Pos          = g_CpPos; g_Fwd = g_CpFwd; g_Up = g_CpUp;
    g_Vel          = g_CpVel;
    g_PitchRate    = g_CpPitchRate;
    g_RollRate     = g_CpRollRate;
    g_YawRateV     = g_CpYawRate;
    g_Throttle     = g_CpThrottle;
    g_MotorThrottle = g_CpMotorThr;
    cam.m_FreeVal_Loc_Translation = g_Pos;
    ComputeCameraAngles(cam);

    if (S_Rec) {
        while (g_TrkT.Length > 0 && g_TrkT[g_TrkT.Length - 1] > g_CpRecTime) {
            g_TrkT.RemoveLast();
            g_TrkPos.RemoveLast();
            g_TrkYaw.RemoveLast();
            g_TrkPitch.RemoveLast();
            g_TrkRoll.RemoveLast();
        }
        g_RecTime = g_CpRecTime;
    }
}

void HandleCpRelease(CGameControlCameraFree@ cam, uint durationMs) {
    if (durationMs >= 900) {
        g_HasCheckpoint = false;          // appui long → efface
    } else if (g_HasCheckpoint) {
        RestoreCheckpoint(cam);           // appui court → retry
    } else {
        SetCheckpoint();                  // appui court sans CP → sauvegarde
    }
}

void ResetToStart(CGameControlCameraFree@ cam) {
    g_Pos    = g_StartPos;
    g_Fwd    = g_StartFwd;
    g_Up     = g_StartUp;
    g_Vel    = vec3(0, 0, 0);
    g_PitchRate = 0; g_RollRate = 0; g_YawRateV = 0;
    g_Throttle = 0; g_MotorThrottle = 0;
    cam.m_FreeVal_Loc_Translation = g_Pos;
    ComputeCameraAngles(cam);

    if (S_Rec && g_HasCheckpoint) {
        while (g_TrkT.Length > 0 && g_TrkT[g_TrkT.Length - 1] > g_CpRecTime) {
            g_TrkT.RemoveLast();
            g_TrkPos.RemoveLast();
            g_TrkYaw.RemoveLast();
            g_TrkPitch.RemoveLast();
            g_TrkRoll.RemoveLast();
        }
        g_RecTime = g_CpRecTime;
    }
}

void TrimTrajectoryTo(float t) {
    uint newLen = 0;
    for (uint i = 0; i < g_TrkT.Length; i++) {
        if (g_TrkT[i] <= t) newLen = i + 1;
        else break;
    }
    g_TrkT.Resize(newLen);
    g_TrkPos.Resize(newLen);
    g_TrkYaw.Resize(newLen);
    g_TrkPitch.Resize(newLen);
    g_TrkRoll.Resize(newLen);
    g_TrkFwd.Resize(newLen);
    g_TrkUp.Resize(newLen);
    g_TrkVel.Resize(newLen);
    g_RecTime = (newLen > 0) ? g_TrkT[newLen - 1] : 0.0f;
}

void RestorePhysicsFromTrajectory() {
    if (g_TrkT.Length == 0) return;
    uint last = g_TrkT.Length - 1;
    g_Pos = g_TrkPos[last];
    if (last < g_TrkFwd.Length) {
        g_Fwd = g_TrkFwd[last];
        g_Up  = g_TrkUp[last];
    }
    g_Vel = (last < g_TrkVel.Length) ? g_TrkVel[last] : vec3(0, 0, 0);
    g_PitchRate     = 0.0f;
    g_RollRate      = 0.0f;
    g_YawRateV      = 0.0f;
    g_Throttle      = 0.0f;
    g_MotorThrottle = 0.0f;
}

// Calcule et stocke les angles TM à partir de l'orientation courante.
// Appelé depuis Main() après chaque mise à jour physique.
void ComputeCameraAngles(CGameControlCameraFree@ cam) {
    vec3 camRight = VCross(g_Fwd, g_Up);
    float tiltRad = S_CameraTilt * DEG2RAD;
    vec3 camFwd = VNorm(RotAround(g_Fwd, camRight, -tiltRad));
    vec3 camUp  = VNorm(RotAround(g_Up,  camRight, -tiltRad));

    g_CamYaw   = Math::Atan2(camFwd.x, camFwd.z);
    g_CamPitch = -Math::Asin(Math::Clamp(camFwd.y, -1.0f, 1.0f));

    if (S_VisualRoll) {
        vec3  wu     = vec3(0, 1, 0);
        float d      = VDot(wu, camFwd);
        vec3  pu_raw = vec3(wu.x - camFwd.x*d, wu.y - camFwd.y*d, wu.z - camFwd.z*d);
        float pu_len = Math::Sqrt(pu_raw.x*pu_raw.x + pu_raw.y*pu_raw.y + pu_raw.z*pu_raw.z);
        if (pu_len > 0.01f) {
            vec3 pu = vec3(pu_raw.x/pu_len, pu_raw.y/pu_len, pu_raw.z/pu_len);
            g_CamRoll = Math::Atan2(VDot(VCross(pu, camUp), camFwd), VDot(pu, camUp))
                        * (S_InvVisualRoll ? -1.0f : 1.0f);
        } else {
            g_CamRoll = 0.0f;
        }
    } else {
        g_CamRoll = 0.0f;
    }

    // Application directe dans UpdatePhysics — RenderUpdate fait une deuxième
    // passe pour écraser l'éventuelle interférence de TM entre les deux.
    cam.m_Yaw   = g_CamYaw;
    cam.m_Pitch = g_CamPitch;
    cam.m_Roll  = g_CamRoll;
}

void UpdatePhysics(CGameControlCameraFree@ cam, CInputScriptPad@ pad, float dt) {

    SuppressTMCamInput(cam);

    // Désactiver les contrôles gamepad natifs TM pour free cam (émule tm-pad-freecam)
    GetApp().SystemConfig.InputsDisableFreeCamPadControl = true;

    // ── Rec / Play ────────────────────────────────────────────
    if (S_Rec && S_Play) S_Play = false; // mutuellement exclusifs

    // Démarrage enregistrement → efface l'ancien
    if (S_Rec && !g_WasRec) {
        g_TrkT.Resize(0); g_TrkPos.Resize(0);
        g_TrkYaw.Resize(0); g_TrkPitch.Resize(0); g_TrkRoll.Resize(0);
        g_TrkFwd.Resize(0); g_TrkUp.Resize(0); g_TrkVel.Resize(0);
        g_RecTime    = 0.0f;
        g_NextKfTime = 0.0f;
    }
    g_WasRec = S_Rec;

    // Démarrage playback → initialise la tête de lecture
    if (S_Play && !g_WasPlay) {
        if (g_TrkT.Length < 2) { S_Play = false; }
        else { g_PlayTime = 0.0f; g_PlayIdx = 0; }
    }
    g_WasPlay = S_Play;

    // ── Rewind (hold-to-rewind, works from any mode) ──────────
    bool rewindDown = IsPadBtnDown(pad, S_RewindPadBtn)
                   || (S_RewindKey > 0 && IsKeyBtnHeld(S_RewindKey));

    if (rewindDown && !g_RewindWasDown && !g_Rewinding && S_Rec) {
        if (g_TrkT.Length >= 2) {
            g_RecBeforeRewind = S_Rec;
            S_Rec  = false;
            S_Play = false;
            g_Rewinding = true;
            g_PlayTime  = g_RecBeforeRewind ? g_RecTime : g_TrkT[g_TrkT.Length - 1];
            g_PlayIdx   = uint(Math::Max(0, int(g_TrkT.Length) - 2));
            while (g_PlayIdx > 0 && g_PlayTime < g_TrkT[g_PlayIdx])
                g_PlayIdx--;
        }
    }

    if (g_Rewinding && rewindDown) {
        g_PlayTime -= dt;
        if (g_PlayTime < 0.0f) g_PlayTime = 0.0f;
        while (g_PlayIdx > 0 && g_PlayTime < g_TrkT[g_PlayIdx])
            g_PlayIdx--;

        uint tLen = g_TrkT.Length;
        uint i0 = g_PlayIdx;
        uint i1 = (i0 + 1 < tLen) ? i0 + 1 : i0;
        float t0 = g_TrkT[i0], t1 = (i0 != i1) ? g_TrkT[i1] : t0 + 0.001f;
        float a  = Math::Clamp((g_PlayTime - t0) / (t1 - t0), 0.0f, 1.0f);

        g_Pos      = g_TrkPos[i0]   + (g_TrkPos[i1]   - g_TrkPos[i0])   * a;
        g_CamYaw   = g_TrkYaw[i0]   + (g_TrkYaw[i1]   - g_TrkYaw[i0])   * a;
        g_CamPitch = g_TrkPitch[i0] + (g_TrkPitch[i1] - g_TrkPitch[i0]) * a;
        g_CamRoll  = g_TrkRoll[i0]  + (g_TrkRoll[i1]  - g_TrkRoll[i0])  * a;
        cam.m_FreeVal_Loc_Translation = g_Pos;
        cam.m_Yaw   = g_CamYaw;
        cam.m_Pitch = g_CamPitch;
        cam.m_Roll  = g_CamRoll;
        g_RewindWasDown = true;
        return;
    }

    if (g_Rewinding && !rewindDown) {
        g_Rewinding = false;
        TrimTrajectoryTo(g_PlayTime);
        RestorePhysicsFromTrajectory();
        if (g_RecBeforeRewind) {
            g_RecTime     = g_PlayTime;
            g_NextKfTime  = g_PlayTime;
            S_Rec         = true;
            g_WasRec      = true;  // évite le fresh-start qui effacerait la trajectoire
        }
    }

    g_RewindWasDown = rewindDown;

    // ── Mode PLAYBACK : bypass total de la physique ───────────
    if (S_Play && g_TrkT.Length >= 2) {
        g_PlayTime += dt;
        uint trkLen = g_TrkT.Length;
        while (g_PlayIdx + 2 < trkLen && g_PlayTime >= g_TrkT[g_PlayIdx + 1])
            g_PlayIdx++;

        float tEnd = g_TrkT[trkLen - 1];
        if (g_PlayTime >= tEnd) { g_PlayTime = tEnd; S_Play = false; }

        uint i0 = g_PlayIdx, i1 = g_PlayIdx + 1;
        float t0 = g_TrkT[i0], t1 = g_TrkT[i1];
        float a  = (t1 > t0) ? Math::Clamp((g_PlayTime - t0) / (t1 - t0), 0.0f, 1.0f) : 0.0f;

        g_Pos      = g_TrkPos[i0]   + (g_TrkPos[i1]   - g_TrkPos[i0])   * a;
        g_CamYaw   = g_TrkYaw[i0]   + (g_TrkYaw[i1]   - g_TrkYaw[i0])   * a;
        g_CamPitch = g_TrkPitch[i0] + (g_TrkPitch[i1] - g_TrkPitch[i0]) * a;
        g_CamRoll  = g_TrkRoll[i0]  + (g_TrkRoll[i1]  - g_TrkRoll[i0])  * a;

        cam.m_FreeVal_Loc_Translation = g_Pos;
        cam.m_Yaw   = g_CamYaw;
        cam.m_Pitch = g_CamPitch;
        cam.m_Roll  = g_CamRoll;
        return;
    }
    // ─────────────────────────────────────────────────────────

    if (!GetApp().InputPort.IsFocused) return;

    if (!g_PosInit) {
        g_Pos = cam.m_FreeVal_Loc_Translation;
        float yaw = cam.m_Yaw;
        g_Fwd = VNorm(vec3(Math::Sin(yaw), 0.0f, Math::Cos(yaw)));
        g_Up  = vec3(0, 1, 0);
        g_PosInit = true;
        g_StartPos = g_Pos;
        g_StartFwd = g_Fwd;
        g_StartUp  = g_Up;
    }

    float rawT = -pad.LeftStickY * (S_InvThrottle ? -1.0f : 1.0f);
    float tNorm = Math::Clamp((rawT + 1.0f) * 0.5f, 0.0f, 1.0f);
    float minThr01 = Math::Clamp(S_MinThrottlePct / 100.0f, 0.0f, 0.95f);
    float tShaped = Math::Pow(tNorm, S_ThrottleCurve);
    g_Throttle = tNorm < 0.001f ? 0.0f : (minThr01 + (1.0f - minThr01) * tShaped) * S_ThrottleMax;

    float yawIn   =  DZ(pad.LeftStickX) * (S_InvYaw   ? -1.0f : 1.0f);
    float pitchIn = -DZ(pad.RightStickY)* (S_InvPitch  ? -1.0f : 1.0f);
    float rollIn  =  DZ(pad.RightStickX)* (S_InvRoll   ? -1.0f : 1.0f);

    // R1 → toggle enregistrement, B → reset position (front montant)
    bool recDown     = IsPadBtnDown(pad, S_RecPadBtn);
    bool respawnDown = IsPadBtnDown(pad, S_RespawnPadBtn);
    bool cpDown      = IsPadBtnDown(pad, S_CpPadBtn);
    if (recDown     && !g_RecWasDown)     S_Rec = !S_Rec;
    if (respawnDown && !g_RespawnWasDown) ResetToStart(cam);
    uint nowCp = GetApp().TimeSinceInitMs;
    if (cpDown  && !g_CpWasDown)  g_CpPadPressMs = nowCp;
    if (!cpDown &&  g_CpWasDown)  HandleCpRelease(cam, nowCp - g_CpPadPressMs);
    g_RecWasDown     = recDown;
    g_RespawnWasDown = respawnDown;
    g_CpWasDown      = cpDown;

    // Motor lag : filtre premier ordre sur le throttle (comme un vrai ESC)
    if (S_MotorTau > 0.0001f)
        g_MotorThrottle += (g_Throttle - g_MotorThrottle) * Math::Min(1.0f, dt / S_MotorTau);
    else
        g_MotorThrottle = g_Throttle;

    float angLerp = 1.0f - Math::Pow(S_AngInertia, dt * 60.0f);
    g_PitchRate += (pitchIn * S_TiltRate - g_PitchRate) * angLerp;
    g_RollRate  += (rollIn  * S_TiltRate - g_RollRate)  * angLerp;
    g_YawRateV  += (yawIn   * S_YawRate  - g_YawRateV)  * angLerp;

    // Gyroscopic coupling — équation d'Euler simplifiée (Izz > Ixx ≈ Iyy)
    // tau_gyro_pitch = (Izz-Iyy)/Ixx * ω_yaw * ω_roll  → perturbation pitch quand yaw+roll
    // tau_gyro_roll  = (Izz-Ixx)/Iyy * ω_yaw * ω_pitch → perturbation roll quand yaw+pitch
    if (S_GyroCoeff > 0.0f) {
        g_PitchRate += S_GyroCoeff * g_YawRateV * g_RollRate  * dt;
        g_RollRate  -= S_GyroCoeff * g_YawRateV * g_PitchRate * dt;
    }

    vec3 right = VCross(g_Fwd, g_Up);

    g_Fwd = VNorm(RotAround(g_Fwd, right,  -g_PitchRate * dt));
    g_Up  = VNorm(RotAround(g_Up,  right,  -g_PitchRate * dt));
    g_Up  = VNorm(RotAround(g_Up,  g_Fwd,  -g_RollRate  * dt));
    g_Fwd = VNorm(RotAround(g_Fwd, g_Up,   -g_YawRateV  * dt));

    g_Fwd = VNorm(g_Fwd);
    g_Up  = VNorm(g_Up - g_Fwd * VDot(g_Up, g_Fwd));

    vec3 force = g_Up * (g_MotorThrottle * g_MotorThrottle) * S_Thrust;
    if (S_GravityEnabled) force.y -= S_Gravity;

    g_Vel += force * dt;

    // Drag linéaire (frottement bas-vitesse)
    g_Vel.x *= Math::Pow(1.0f - S_Drag,  dt * 60.0f);
    g_Vel.z *= Math::Pow(1.0f - S_Drag,  dt * 60.0f);
    g_Vel.y *= Math::Pow(1.0f - S_DragY, dt * 60.0f);

    // Drag quadratique (proportionnel à v²) — cap naturel de vitesse
    float hspd = Math::Sqrt(g_Vel.x*g_Vel.x + g_Vel.z*g_Vel.z);
    if (hspd > 0.001f) {
        float qf = Math::Min(1.0f, S_QuadDrag * hspd * dt);
        g_Vel.x *= (1.0f - qf);
        g_Vel.z *= (1.0f - qf);
    }
    float vspd = Math::Abs(g_Vel.y);
    if (vspd > 0.001f)
        g_Vel.y *= Math::Max(0.0f, 1.0f - S_QuadDragY * vspd * dt);

    g_Pos += g_Vel * dt;
    if (S_UseMinHeight && g_Pos.y < S_MinHeight) {
        g_Pos.y = S_MinHeight;
        if (g_Vel.y < 0.0f) g_Vel.y = 0.0f;
    }

    // Applique immédiatement la position (sécurité contre les interférences)
    cam.m_FreeVal_Loc_Translation = g_Pos;

    // Pré-calcul des angles — écrasés proprement dans RenderUpdate()
    ComputeCameraAngles(cam);

    // Enregistrement de la frame courante
    if (S_Rec) {
        float interval = 1.0f / S_KfFreq;
        while (g_RecTime >= g_NextKfTime) {
            g_TrkT.InsertLast(g_NextKfTime);
            g_TrkPos.InsertLast(g_Pos);
            g_TrkYaw.InsertLast(g_CamYaw);
            g_TrkPitch.InsertLast(g_CamPitch);
            g_TrkRoll.InsertLast(g_CamRoll);
            g_TrkFwd.InsertLast(g_Fwd);
            g_TrkUp.InsertLast(g_Up);
            g_TrkVel.InsertLast(g_Vel);
            g_NextKfTime += interval;
        }
        g_RecTime += dt;
    }
}

// ── BOUCLE PRINCIPALE (physique) ─────────────────────────────

void Main() {
    LoadMisc();
    InitLoc();
    LoadPhysics();
    LoadBinds();
    while (true) {
        yield();

        if (!S_Enabled) {
            if (g_WasOn) { ResetPhysics(); g_WasOn = false; }
            g_LastPhysUpdateMs = 0;
            g_FilteredDt = 1.0f / 120.0f;
            GetApp().SystemConfig.InputsDisableFreeCamPadControl = false;
            continue;
        }

        auto app = GetApp();
        if (app is null || app.Viewport is null) continue;

        uint nowMs = app.TimeSinceInitMs;
        float dtRaw = 1.0f / 120.0f;
        if (g_LastPhysUpdateMs != 0 && nowMs >= g_LastPhysUpdateMs) {
            dtRaw = float(nowMs - g_LastPhysUpdateMs) * 0.001f;
        }
        g_LastPhysUpdateMs = nowMs;
        if (dtRaw <= 0.0f) dtRaw = 1.0f / 240.0f;
        dtRaw = Math::Min(dtRaw, 0.05f);

        // 1st-order smoothing on dt reduces visible vertical jitter from millisecond timer quantization.
        const float dtAlpha = 0.25f;
        g_FilteredDt += (dtRaw - g_FilteredDt) * dtAlpha;
        float dt = g_FilteredDt;

        auto cam = GetFreeCam();
        if (cam is null) {
            if (g_WasOn) { ResetPhysics(); g_WasOn = false; }
            g_LastPhysUpdateMs = 0;
            g_FilteredDt = 1.0f / 120.0f;
            continue;
        }

        auto pad = GetGamepad();
        if (pad is null) {
            if (g_WasOn) { ResetPhysics(); g_WasOn = false; }
            g_LastPhysUpdateMs = 0;
            g_FilteredDt = 1.0f / 120.0f;
            continue;
        }

        UpdatePhysics(cam, pad, dt);
        g_WasOn = true;
    }
}

// ── APPLICATION CAMÉRA (juste avant le rendu) ─────────────────
//
// RenderUpdate() s'exécute chaque frame de rendu, APRÈS que TM
// a traité ses propres inputs (joystick droit → pitch/yaw caméra).
// En écrivant ici, nos valeurs écrasent celles de TM en dernier.

void RenderUpdate() {
    if (!S_Enabled || !g_WasOn) return;
    auto cam = GetFreeCam();
    if (cam is null) return;

    SuppressTMCamInput(cam);  // double passe : couvre les frames sans tick physique
    cam.m_Yaw   = g_CamYaw;
    cam.m_Pitch = g_CamPitch;
    cam.m_Roll  = g_CamRoll;
    cam.m_FreeVal_Loc_Translation = g_Pos;

    // Binds clavier (front montant géré par IsKeyPressed(false))
    if (IsKeyBtnDown(S_RecKey))     S_Rec = !S_Rec;
    if (IsKeyBtnDown(S_RespawnKey)) ResetToStart(cam);
    // Rewind clavier géré dans UpdatePhysics via IsKeyBtnHeld
    if (S_CpKey > 0) {
        bool keyHeld = IsKeyBtnHeld(S_CpKey);
        uint nowK    = GetApp().TimeSinceInitMs;
        if ( keyHeld && !g_CpKeyWasDown) g_CpKeyPressMs = nowK;
        if (!keyHeld &&  g_CpKeyWasDown) HandleCpRelease(cam, nowK - g_CpKeyPressMs);
        g_CpKeyWasDown = keyHeld;
    }
}

// ── OVERLAY STICKS ────────────────────────────────────────────

void DrawStickBox(float x, float y, float sx, float sy) {
    float sz = g_OvSz;
    float cx = x + sz * 0.5f;
    float cy = y + sz * 0.5f;
    float r  = sz * 0.42f;

    nvg::BeginPath();
    nvg::Rect(x, y, sz, sz);
    nvg::FillColor(vec4(0, 0, 0, g_OvBgAlpha));
    nvg::Fill();

    nvg::BeginPath();
    nvg::Rect(x, y, sz, sz);
    nvg::StrokeColor(vec4(1, 1, 1, g_OvBorderAlpha));
    nvg::StrokeWidth(1.0f);
    nvg::Stroke();

    nvg::BeginPath();
    nvg::MoveTo(vec2(cx, y + 3));
    nvg::LineTo(vec2(cx, y + sz - 3));
    nvg::StrokeColor(vec4(1, 1, 1, g_OvCrossAlpha));
    nvg::StrokeWidth(1.0f);
    nvg::Stroke();

    nvg::BeginPath();
    nvg::MoveTo(vec2(x + 3, cy));
    nvg::LineTo(vec2(x + sz - 3, cy));
    nvg::Stroke();

    nvg::BeginPath();
    nvg::Circle(vec2(cx + sx * r, cy + sy * r), g_OvDotSize);
    nvg::FillColor(vec4(g_OvDotR, g_OvDotG, g_OvDotB, 1.0f));
    nvg::Fill();
}

void DrawTriggerBar(float x, float y, float w, float val, bool active) {
    float h = 7.0f;
    nvg::BeginPath();
    nvg::Rect(x, y, w, h);
    nvg::FillColor(vec4(0, 0, 0, 0.6f));
    nvg::Fill();

    if (val > 0.001f) {
        nvg::BeginPath();
        nvg::Rect(x, y, val * w, h);
        nvg::FillColor(active ? vec4(1.0f, 0.45f, 0.1f, 0.9f) : vec4(0.35f, 0.85f, 0.35f, 0.75f));
        nvg::Fill();
    }

    nvg::BeginPath();
    nvg::Rect(x, y, w, h);
    nvg::StrokeColor(vec4(1, 1, 1, 0.25f));
    nvg::StrokeWidth(1.0f);
    nvg::Stroke();
}


void Render() {
    if (!S_Enabled || !g_WasOn) return;

    if (!S_ShowInputOverlay) return;
    auto pad = GetGamepad();
    if (pad is null) return;

    float bx  = float(S_OverlayX);
    float by  = float(S_OverlayY);

    DrawStickBox(bx,                    by, pad.LeftStickX,  pad.LeftStickY);
    DrawStickBox(bx + g_OvSz + g_OvGap, by, pad.RightStickX, pad.RightStickY);

    if (g_ShowLiveValues && g_WasOn) {
        float pitchDeg = -Math::Asin(Math::Clamp(g_Fwd.y, -1.0f, 1.0f)) * RAD2DEG;
        float rollDeg  = g_CamRoll  * RAD2DEG;
        float yawDeg   = Math::Atan2(g_Fwd.x, g_Fwd.z) * RAD2DEG;

        float tx = bx;
        float ty = by + g_OvSz + 14.0f;
        nvg::FontSize(12.0f);
        nvg::FillColor(vec4(1.0f, 1.0f, 1.0f, 0.85f));
        nvg::Text(tx, ty,
            "P " + Text::Format("%+.0f", pitchDeg) + "°"
            + "  R " + Text::Format("%+.0f", rollDeg)  + "°"
            + "  Y " + Text::Format("%+.0f", yawDeg)   + "°"
            + "  T " + Text::Format("%.0f", g_Throttle * 100.0f) + "%");
    }
}

// ── MENU ET HUD ───────────────────────────────────────────────

void RenderMenu() {
    if (UI::MenuItem("FPV Drone" + (S_Enabled ? "  ● " + T(LS_STATUS_ON) : "  ○ " + T(LS_STATUS_OFF)), "", S_Enabled)) {
        S_Enabled = !S_Enabled;
    }
}

void RenderInterface() {
    if (!S_Enabled) return;

    int wf = UI::WindowFlags::NoTitleBar  | UI::WindowFlags::NoResize
           | UI::WindowFlags::NoMove      | UI::WindowFlags::NoBackground
           | UI::WindowFlags::NoScrollbar;

    // HUD FPV — uniquement quand le drone est actif
    if (S_Enabled && g_WasOn) {
        UI::SetNextWindowPos(16, 470, UI::Cond::Always);
        UI::SetNextWindowSize(360, 66, UI::Cond::Always);
        UI::Begin("##fpvhud", wf);

        float speed = Math::Sqrt(g_Vel.x*g_Vel.x + g_Vel.y*g_Vel.y + g_Vel.z*g_Vel.z);
        int   thr   = int(g_Throttle * 100.0f);
        float pitchDeg = -Math::Asin(Math::Clamp(g_Fwd.y, -1.0f, 1.0f)) / DEG2RAD;

        uint trkLen = g_TrkT.Length;
        float trkDur = (trkLen > 0) ? g_TrkT[trkLen - 1] : 0.0f;
        string recStatus = "";
        if (g_Rewinding && trkLen > 0)
            recStatus = "  \\$F80⏪ REWIND " + Text::Format("%.1f", g_PlayTime) + "s / "
                      + Text::Format("%.1f", trkDur) + "s";
        else if (S_Rec)
            recStatus = "  \\$F00⏺ REC " + trkLen + " fr (" + Text::Format("%.1f", g_RecTime) + "s)";
        else if (S_Play && trkLen > 0)
            recStatus = "  \\$0F0▶ PLAY " + Text::Format("%.1f", g_PlayTime) + "s / "
                      + Text::Format("%.1f", trkDur) + "s";
        else if (trkLen > 0)
            recStatus = "  \\$888" + trkLen + " fr enreg.";

        string cpStatus = g_HasCheckpoint ? "  \\$FF0⬟ CP" : "";
        UI::Text("\\$0F0" + T(LS_DRONE_ON) + "  " + T(LS_CAM7) + recStatus + cpStatus);
        UI::Text(T(LS_SPEED_LBL) + Text::Format("%.1f", speed) + " u/s"
               + "   " + T(LS_THR_LBL) + thr + "%"
               + "   " + T(LS_GRAV_LBL) + (S_GravityEnabled ? "\\$F80" + T(LS_ON_LBL) : "\\$888" + T(LS_OFF_LBL)));
        UI::Text("Pitch " + Text::Format("%+.1f", pitchDeg) + "°"
               + "   ↑" + Text::Format("%.1f", g_Vel.y) + " u/s"
               + "   Tilt cam : " + Text::Format("%.0f", S_CameraTilt) + "°");
        UI::End();
    }

    // Panneau export CSV — toujours visible
    uint trkLen2 = g_TrkT.Length;
    {
        int yPos = (S_Enabled && g_WasOn) ? 540 : 470;
        UI::SetNextWindowPos(16, yPos, UI::Cond::Always);
        UI::SetNextWindowSize(400, 48, UI::Cond::Always);
        UI::Begin("##fpvmt", wf);

        if (trkLen2 >= 2) {
            float dur = g_TrkT[trkLen2 - 1];
            UI::Text("\\$888Traj : " + trkLen2 + " fr / "
                   + Text::Format("%.1f", dur) + "s");
            UI::SameLine();
        }

        if (UI::Button("→ CSV")) { ExportToCSV(); }

        if (g_ExportStatus.Length > 0) {
            string col = g_ExportError ? "\\$F00✗ " : "\\$0F0✓ ";
            UI::Text(col + g_ExportStatus);
        }

        UI::End();
    }

    // ── Presets ──────────────────────────────────────────────────
    {
        if (!g_PresetsLoaded) {
            g_PresetsLoaded = true;
            for (int s = 0; s < NUM_PRESETS; s++)
                g_PresetNames[s] = ReadPresetName(s);
        }
        int yPre = (S_Enabled && g_WasOn) ? 592 : 522;
        UI::SetNextWindowPos(16, yPre, UI::Cond::Always);
        UI::SetNextWindowSize(400, 112, UI::Cond::Always);
        UI::Begin("##fpvpresets", wf);
        UI::Text(T(LS_PRESETS_LBL));
        for (int s = 0; s < NUM_PRESETS; s++) {
            UI::PushID(s);
            string pname = g_PresetNames[s];
            UI::SetNextItemWidth(160);
            g_PresetNames[s] = UI::InputText("##pn" + s, pname);
            UI::SameLine();
            if (UI::Button(T(LS_SAVE_LBL) + "##sv"))   SavePreset(s);
            UI::SameLine();
            if (UI::Button(T(LS_LOAD_LBL) + "##ld")) LoadPreset(s);
            UI::PopID();
        }
        if (g_PresetStatus.Length > 0)
            UI::Text((g_PresetError ? "\\$F00✗ " : "\\$0F0✓ ") + g_PresetStatus);
        UI::End();
    }


}

void OnDisabled() {
    ResetPhysics();
    g_WasOn = false;
    g_LastPhysUpdateMs = 0;
    g_FilteredDt = 1.0f / 120.0f;
    GetApp().SystemConfig.InputsDisableFreeCamPadControl = false;
}
