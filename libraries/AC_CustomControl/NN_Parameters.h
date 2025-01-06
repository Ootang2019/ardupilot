#pragma once
#ifndef __NN_PARAMETERS_DEF_H__
#define __NN_PARAMETERS_DEF_H__

#include <vector>

namespace NN {


static constexpr bool USE_LAYERNORM = 0;
static constexpr int N_ACT = 3;
static constexpr int N_STATE = 6;
static constexpr int N_GOAL = 3;
static constexpr int N_TASK = 7;
static constexpr int N_HIDDEN = 16;

static constexpr float AVEL_LIM = 15.700000;
static constexpr float POS_LIM = 999999.900000;
static constexpr float AUTHORITY = 0.250000;

std::vector<float> OBS = {
0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, };
std::vector<float> TASK = {
0.250000, 0.250000, 0.250000, 0.250000, 0.000000, 0.000000, 0.000000, };
static const std::vector<float> A = {
    1.0f,
    -0.6446180939674377f,
    -0.011116204783320427f,
    1.0f,
    -0.030000125989317894f,
    -0.044383373111486435f,
    -0.9089627265930176f,
    -0.5283390879631042f,
    0.008669078350067139f,
    0.5350884795188904f,
    0.5149133801460266f,
    -0.41985514760017395f,
    -0.42933595180511475f,
    0.5604193210601807f,
    -0.44350117444992065f,
    0.538949191570282f,
    0.41627469658851624f,
    -0.4704020023345947f,
    -0.4894588887691498f,
    -0.5831834673881531f,
    1.0f,
    0.020445566624403f,
    -0.018780168145895004f,
    1.0f,
    0.15226902067661285f,
    -0.5423934459686279f,
    -1.0f,
    -0.0038635525852441788f,
    0.5305726528167725f,
    0.5057479739189148f,
    -0.4204499423503876f,
    -0.4405643939971924f,
    0.5270808339118958f,
    -0.47124627232551575f,
    0.49954450130462646f,
    -0.4278453588485718f,
    0.46489304304122925f,
    -0.4297558069229126f,
    -0.4220220446586609f,
    -0.43904784321784973f,
    0.0902937576174736f,
    -0.048658549785614014f,
    -0.042611412703990936f,
    1.0f,
    -0.41933050751686096f,
    -0.42253175377845764f,
    -0.9624981880187988f,
    -0.5311973094940186f,
    -0.5512556433677673f,
    0.4811657667160034f,
    0.43377822637557983f,
    0.4360937476158142f,
    -0.5530017614364624f,
    0.42260751128196716f,
    -0.33853843808174133f,
    -0.32708871364593506f,
    0.5879745483398438f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.5404805541038513f, 
    0.35299763083457947f, 
    0.12526896595954895f, 
    -0.19644911587238312f, 
    0.12473344802856445f, 
    0.3261926472187042f, 
    -0.33209073543548584f, 
    -0.4527933895587921f, 
    -0.24510012567043304f, 
    -0.28414231538772583f, 
    -0.06511067599058151f, 
    0.4189673364162445f, 
    0.4676226079463959f, 
    0.3426051139831543f, 
    -0.37490564584732056f, 
    -0.25519391894340515f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.02581733837723732f,
    -0.03458636999130249f,
    -0.11157502233982086f,
    0.07255730777978897f,
    -0.070001520216465f,
    0.0340321809053421f,
    -0.16473804414272308f,
    -0.023340320214629173f,
    -0.054663266986608505f,
    0.042000994086265564f,
    -0.11812804639339447f,
    0.25220268964767456f,
    -0.1172318235039711f,
    -0.08354245126247406f,
    -0.05835548788309097f,
    0.00927695445716381f,
};

static const std::vector<float> ANG_EMB_W = {
    -0.6706017255783081f, 
    0.7822420597076416f, 
    -0.15435995161533356f, 
    1.04825758934021f, 
    -0.990961492061615f, 
    -0.4464797377586365f, 
    0.04580332711338997f, 
    0.9418522119522095f, 
    -1.4233413934707642f, 
    -0.2674354314804077f, 
    0.47409966588020325f, 
    0.47839468717575073f, 
    -0.10731763392686844f, 
    0.5451777577400208f, 
    0.8009079098701477f, 
    -1.384083867073059f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.037406567484140396f,
    -0.013773874379694462f,
    -0.09587444365024567f,
    0.06402342766523361f,
    -0.06874357908964157f,
    0.048218656331300735f,
    -0.12037819623947144f,
    -0.032857440412044525f,
    -0.05191345512866974f,
    0.03320129215717316f,
    -0.1055229976773262f,
    0.16836819052696228f,
    -0.10668811947107315f,
    -0.0769076868891716f,
    -0.060670915991067886f,
    0.016011236235499382f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    -2.1311917304992676f, 
    0.3716677129268646f, 
    -0.6310710310935974f, 
    1.8731836080551147f, 
    -2.194347381591797f, 
    -0.7014656066894531f, 
    0.7165477275848389f, 
    2.604626178741455f, 
    -1.6470720767974854f, 
    -1.3685379028320312f, 
    0.5506694316864014f, 
    0.12212670594453812f, 
    -1.2835910320281982f, 
    1.2198500633239746f, 
    1.435173749923706f, 
    -2.906663179397583f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.031697794795036316f,
    -0.04010559245944023f,
    0.03864729404449463f,
    0.026691023260354996f,
    -0.03174395114183426f,
    0.006976154632866383f,
    0.04265981912612915f,
    -0.017972415313124657f,
    -0.018085675314068794f,
    0.03036934696137905f,
    0.019679497927427292f,
    -0.07358986884355545f,
    0.005094907712191343f,
    -0.0008001200621947646f,
    -0.03874120116233826f,
    0.002872452372685075f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.3605366349220276f, 
    -0.27292269468307495f, 
    0.34463608264923096f, 
    -0.12521791458129883f, 
    0.33847957849502563f, 
    -0.05959160998463631f, 
    0.06307964771986008f, 
    -0.5857712626457214f, 
    -0.33389297127723694f, 
    0.12580065429210663f, 
    0.09272215515375137f, 
    -0.453698992729187f, 
    0.048078130930662155f, 
    0.3910485804080963f, 
    -0.08911987394094467f, 
    -0.28659820556640625f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.025316253304481506f,
    0.029908888041973114f,
    0.10360828042030334f,
    -0.06748399883508682f,
    0.07090044021606445f,
    -0.0315973125398159f,
    0.16699059307575226f,
    0.024808363988995552f,
    0.0515294149518013f,
    -0.04437049850821495f,
    0.11362963169813156f,
    -0.25280138850212097f,
    0.11301996558904648f,
    0.08123154938220978f,
    0.057290855795145035f,
    -0.008966934867203236f,
};

static const std::vector<float> GCN0_W = {
    -0.13126111030578613f,     0.2077159434556961f,     -0.08347585797309875f,     -0.16859224438667297f,     -0.41699954867362976f,     -0.2422122359275818f,     0.473706990480423f,     0.3266436457633972f,     -0.565244197845459f,     -0.30946990847587585f,     0.0995984748005867f,     -0.051106397062540054f,     0.021419918164610863f,     0.4689692556858063f,     0.4365856945514679f,     -0.009078159928321838f, 
    -0.4577173590660095f,     0.2264125943183899f,     0.3485105037689209f,     -0.3856019079685211f,     -0.18311817944049835f,     -0.2973769009113312f,     -0.24102753400802612f,     0.08562169224023819f,     -0.12761211395263672f,     -0.12775562703609467f,     0.17455345392227173f,     -0.37962910532951355f,     -0.08115477859973907f,     0.017820175737142563f,     0.41952627897262573f,     0.24183671176433563f, 
    0.4233754873275757f,     -0.5717489123344421f,     0.05661305785179138f,     -0.27263036370277405f,     -0.10806301981210709f,     0.7612120509147644f,     -0.17144350707530975f,     -0.2936018109321594f,     0.6375803351402283f,     -0.14473827183246613f,     -1.5279265642166138f,     0.19263409078121185f,     -0.3579256534576416f,     -1.2089815139770508f,     -0.833881676197052f,     0.4333665668964386f, 
    -0.5350746512413025f,     -0.3477475047111511f,     0.2520356774330139f,     -0.21977847814559937f,     0.19793221354484558f,     -0.2493303269147873f,     0.42759931087493896f,     0.3960908353328705f,     0.37184879183769226f,     0.0005078577087260783f,     -0.07704956829547882f,     -0.4548632800579071f,     0.1805337369441986f,     0.14358049631118774f,     -0.2833714783191681f,     -0.30863577127456665f, 
    0.286598801612854f,     0.1402767300605774f,     0.4816510081291199f,     -0.33104830980300903f,     0.5271444916725159f,     0.348937064409256f,     0.4184072017669678f,     -0.7298872470855713f,     0.5418614745140076f,     -0.13068214058876038f,     0.3536507785320282f,     0.008057462982833385f,     0.08660586178302765f,     -0.23584799468517303f,     -0.03755475580692291f,     1.102725863456726f, 
    0.1316910982131958f,     -0.12598450481891632f,     0.15037290751934052f,     0.3513491451740265f,     0.13516412675380707f,     0.3444942533969879f,     0.04625830054283142f,     0.1400458812713623f,     -0.2665877342224121f,     -0.430258184671402f,     0.5259564518928528f,     -0.09169803559780121f,     -0.10268304497003555f,     -0.03736267238855362f,     0.2559214234352112f,     -0.6741133332252502f, 
    0.34253931045532227f,     -0.11394777148962021f,     0.540429413318634f,     -0.281139999628067f,     0.28755250573158264f,     -0.26983967423439026f,     0.45328813791275024f,     -0.25814658403396606f,     0.4819510281085968f,     0.014426397159695625f,     0.4379720687866211f,     -0.3782728910446167f,     0.5976539254188538f,     0.3696332573890686f,     0.3113211989402771f,     -0.20003943145275116f, 
    -0.23792679607868195f,     0.1648191660642624f,     -0.2614092528820038f,     0.649285078048706f,     -0.23044826090335846f,     0.12902790307998657f,     0.19425106048583984f,     0.22874340415000916f,     -0.49370718002319336f,     -0.2669833302497864f,     0.5728638172149658f,     -0.36135390400886536f,     -0.0034049300011247396f,     0.5142006874084473f,     0.66327965259552f,     -0.9213489294052124f, 
    -0.016506047919392586f,     0.2896535098552704f,     0.47409921884536743f,     -0.007500506937503815f,     -0.10599402338266373f,     -0.3415837585926056f,     -0.0848187506198883f,     0.3513917326927185f,     0.12767314910888672f,     -0.309575617313385f,     0.08251238614320755f,     -0.6405149102210999f,     0.39019665122032166f,     -0.027933668345212936f,     -0.3968534469604492f,     -0.057840608060359955f, 
    0.4922817349433899f,     0.14954717457294464f,     -0.07535789906978607f,     -0.4437178671360016f,     0.47402316331863403f,     0.34800612926483154f,     0.2919168174266815f,     -0.41200417280197144f,     0.3989998400211334f,     0.3263683021068573f,     0.1070530116558075f,     -0.2127581387758255f,     0.17261295020580292f,     0.1565413773059845f,     -0.3425564765930176f,     0.5394347310066223f, 
    -0.30458158254623413f,     -0.36840686202049255f,     0.15058548748493195f,     -0.44345003366470337f,     0.30295076966285706f,     -0.3855918347835541f,     0.013869460672140121f,     0.5593451261520386f,     0.29754212498664856f,     -0.2971755862236023f,     0.052004821598529816f,     0.1977933943271637f,     0.21787579357624054f,     -0.1738697737455368f,     0.06661826372146606f,     -0.39717811346054077f, 
    -0.018074242398142815f,     -0.4322453439235687f,     -0.04694456234574318f,     -0.43644118309020996f,     0.3694016933441162f,     -0.1085139662027359f,     0.0810517743229866f,     -0.4410969614982605f,     0.5204492807388306f,     -0.06894920766353607f,     -0.3105742633342743f,     -0.2855994403362274f,     0.5152209401130676f,     0.13419893383979797f,     -0.007293279282748699f,     1.1355494260787964f, 
    -0.0488128662109375f,     0.48364028334617615f,     0.1424587517976761f,     0.14167974889278412f,     -0.46258488297462463f,     0.12539395689964294f,     0.46144866943359375f,     0.34648221731185913f,     -0.6307430267333984f,     -0.13383346796035767f,     0.3777078688144684f,     -0.18094104528427124f,     -0.17293524742126465f,     -0.012974122539162636f,     0.10616083443164825f,     -0.8054965734481812f, 
    -0.007343747187405825f,     0.36504459381103516f,     0.4699535369873047f,     0.12407182157039642f,     0.11980495601892471f,     0.24807098507881165f,     0.45619142055511475f,     0.165932297706604f,     0.24281559884548187f,     0.12148822098970413f,     -0.05091123282909393f,     -0.15690931677818298f,     0.27945858240127563f,     0.2848762273788452f,     0.034864600747823715f,     0.3199916481971741f, 
    0.5846406817436218f,     -0.020146917551755905f,     0.2577704191207886f,     -0.038468316197395325f,     0.12478811293840408f,     0.08252150565385818f,     -0.08190718293190002f,     -0.8643472790718079f,     0.4449511468410492f,     -0.3395159840583801f,     0.23834215104579926f,     -0.3009105622768402f,     0.2131015956401825f,     -0.2828110456466675f,     -0.05715895816683769f,     0.3540893793106079f, 
    0.10525526106357574f,     -0.03014323301613331f,     -0.29995644092559814f,     0.4057067930698395f,     -0.2828071415424347f,     0.07474800944328308f,     0.1688554733991623f,     -0.45789089798927307f,     -0.3320409059524536f,     0.1966722011566162f,     -0.15119683742523193f,     0.002850560238584876f,     0.2029941827058792f,     -0.31771036982536316f,     0.05764355883002281f,     -0.23455654084682465f, 
};

static const std::vector<float> GCN0_B = {
    0.048339154571294785f,
    0.10317855328321457f,
    -0.20830383896827698f,
    0.11174032837152481f,
    0.06386226415634155f,
    -0.004428722430020571f,
    0.17796052992343903f,
    0.05621775984764099f,
    0.1468181014060974f,
    0.07018154114484787f,
    0.09378281980752945f,
    0.04124128818511963f,
    0.07657502591609955f,
    0.1267160177230835f,
    0.05608342960476875f,
    -0.048943888396024704f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.056839849799871445f,     -0.034430358558893204f,     0.0040390994399785995f,     0.015245513059198856f,     0.1760534793138504f,     -0.07664024084806442f,     0.07326560467481613f,     -0.004613699857145548f,     0.0699315145611763f,     0.16600720584392548f,     0.058458104729652405f,     0.025674615055322647f,     0.00873921625316143f,     -0.0031961596105247736f,     -0.005002225749194622f,     -0.04848417639732361f, 
    -0.05526142194867134f,     0.1779133826494217f,     -0.19231422245502472f,     0.18040430545806885f,     0.22046306729316711f,     -0.048105400055646896f,     0.07580774277448654f,     -0.3261127173900604f,     -0.07223571091890335f,     0.1439490020275116f,     0.12507793307304382f,     0.011018017306923866f,     -0.3824315369129181f,     0.12034431844949722f,     0.18684184551239014f,     0.07374179363250732f, 
    -0.09006459265947342f,     0.13956263661384583f,     -0.15000782907009125f,     0.20196709036827087f,     0.23776006698608398f,     -0.13487139344215393f,     0.010735786519944668f,     -0.12354287505149841f,     0.05069201812148094f,     0.10992700606584549f,     -0.055295560508966446f,     0.20503564178943634f,     -0.1537325382232666f,     0.1666201949119568f,     -0.007153174374252558f,     -0.07730399072170258f, 
    -0.07757966220378876f,     0.03766129910945892f,     0.0036788773722946644f,     -0.10415462404489517f,     0.017279263585805893f,     -0.016419079154729843f,     0.0187346450984478f,     -0.06860479712486267f,     0.04440900310873985f,     -0.0346171073615551f,     -0.008784951642155647f,     -0.11141153424978256f,     0.08522596955299377f,     0.0821017175912857f,     -0.12874285876750946f,     -0.04781845211982727f, 
    0.10471731424331665f,     0.05936655029654503f,     -0.07412794977426529f,     0.09375619143247604f,     -0.33880090713500977f,     -0.07498553395271301f,     -0.024903669953346252f,     0.17363691329956055f,     0.13344356417655945f,     -0.2494446188211441f,     0.08475859463214874f,     -0.257434606552124f,     -0.0664399266242981f,     0.06815929710865021f,     -0.17324109375476837f,     -0.07263904809951782f, 
    -0.40662881731987f,     -0.12819795310497284f,     0.21658624708652496f,     -0.12038420140743256f,     0.09612078219652176f,     -0.030519383028149605f,     -0.031225690618157387f,     -0.11116190254688263f,     -0.24875645339488983f,     -0.009881516918540001f,     0.05892610922455788f,     0.11400704830884933f,     0.019586704671382904f,     0.02945876494050026f,     0.0461052767932415f,     -0.08508475124835968f, 
    0.18584074079990387f,     0.14280909299850464f,     0.03390203416347504f,     0.07954154908657074f,     -0.27085769176483154f,     0.08663847297430038f,     0.14733709394931793f,     0.03338088095188141f,     0.07719329744577408f,     -0.34012719988822937f,     0.06415971368551254f,     -0.3459937572479248f,     0.2273842841386795f,     0.14876040816307068f,     -0.13285328447818756f,     -0.022310109809041023f, 
    0.04829999431967735f,     0.22334113717079163f,     -0.09970535337924957f,     0.06570927798748016f,     -0.19992293417453766f,     0.012035222724080086f,     0.0846974328160286f,     0.27436143159866333f,     0.08439205586910248f,     -0.20415234565734863f,     0.1593984067440033f,     -0.30857086181640625f,     0.17935584485530853f,     0.002845731098204851f,     -0.3360276222229004f,     -0.06519532203674316f, 
    -0.12906034290790558f,     0.17411351203918457f,     -0.33656609058380127f,     0.006891084369271994f,     0.21123148500919342f,     -0.10458558052778244f,     0.16526158154010773f,     -0.2982666790485382f,     0.19802230596542358f,     0.2576918601989746f,     0.09756282716989517f,     0.23877114057540894f,     -0.1685754805803299f,     0.10904831439256668f,     0.23794317245483398f,     0.01939915493130684f, 
    -0.40673190355300903f,     -0.03402763232588768f,     0.2259194254875183f,     -0.09570104628801346f,     0.03534048795700073f,     0.05516662448644638f,     -0.11476932466030121f,     -0.07520041614770889f,     -0.0906897708773613f,     0.058671679347753525f,     -0.043554291129112244f,     0.14505994319915771f,     0.12109750509262085f,     -0.06163544952869415f,     0.06774424016475677f,     -0.08650811016559601f, 
    -0.16829460859298706f,     0.0320291630923748f,     -0.07407832890748978f,     0.05256107822060585f,     -0.049620069563388824f,     -0.18527545034885406f,     0.006979580502957106f,     -0.35580185055732727f,     0.09671157598495483f,     0.08700665086507797f,     0.08873648941516876f,     -0.05192435532808304f,     -0.14976546168327332f,     0.13135023415088654f,     0.138212651014328f,     -0.06661781668663025f, 
    0.08682399988174438f,     -0.031059211120009422f,     -0.06297577917575836f,     -0.28565239906311035f,     -0.12319941818714142f,     0.123428575694561f,     -0.25835737586021423f,     0.13194581866264343f,     -0.00036525301402434707f,     -0.09932081401348114f,     -0.1472448706626892f,     0.03489775210618973f,     0.1337255835533142f,     -0.324302613735199f,     -0.10341676324605942f,     0.13724859058856964f, 
    0.1863909363746643f,     -0.07079067081212997f,     0.07525572925806046f,     0.1185973584651947f,     -0.2918814420700073f,     0.20732691884040833f,     0.16981074213981628f,     0.20631298422813416f,     0.13756221532821655f,     -0.20833122730255127f,     0.06152971461415291f,     -0.12839509546756744f,     0.22103296220302582f,     -0.015178433619439602f,     -0.14650267362594604f,     0.06380151212215424f, 
    0.014162182807922363f,     -0.021597949787974358f,     0.03668135404586792f,     -0.16092561185359955f,     -0.16381263732910156f,     0.12092333287000656f,     -0.2590024769306183f,     0.14487223327159882f,     -0.017810789868235588f,     0.04318894445896149f,     -0.3152666389942169f,     0.09903720021247864f,     0.15035279095172882f,     -0.26743024587631226f,     -0.10422439128160477f,     0.04636704549193382f, 
    -0.1107080951333046f,     -0.0290504302829504f,     0.07289538532495499f,     0.11819253861904144f,     0.07328275591135025f,     0.07083624601364136f,     -0.0446927547454834f,     -0.12259110808372498f,     -0.10850361734628677f,     -0.12108911573886871f,     -0.04931456968188286f,     -0.07572577148675919f,     0.11054770648479462f,     -0.09879406541585922f,     -0.0037078391760587692f,     -0.06002030149102211f, 
    0.09994804114103317f,     0.07142288237810135f,     -0.24182814359664917f,     -0.022769426926970482f,     0.18584854900836945f,     -0.20771117508411407f,     0.07515446096658707f,     -0.32480818033218384f,     0.11068042367696762f,     0.17953555285930634f,     0.03448862209916115f,     0.2483358383178711f,     -0.21595069766044617f,     0.16953682899475098f,     0.21299253404140472f,     0.06528440117835999f, 
    0.19453756511211395f,     -0.04432301223278046f,     -0.04955335706472397f,     0.12323929369449615f,     -0.10590402036905289f,     0.2181200236082077f,     0.017831962555646896f,     0.06304106116294861f,     0.1344563066959381f,     -0.09285536408424377f,     0.16927939653396606f,     -0.25858867168426514f,     0.1404796838760376f,     0.10356297343969345f,     -0.15300185978412628f,     -0.12735244631767273f, 
    0.21490901708602905f,     0.04477526247501373f,     0.2676313817501068f,     0.11276561766862869f,     -0.2683086395263672f,     0.14941097795963287f,     0.0958930179476738f,     0.2170037180185318f,     0.20765233039855957f,     -0.30217108130455017f,     0.027550283819437027f,     -0.12458992749452591f,     0.15977175533771515f,     0.1815924495458603f,     -0.166339710354805f,     -0.0933266282081604f, 
    0.01494495291262865f,     0.07011696696281433f,     -0.031086711212992668f,     0.012617110274732113f,     -0.23712438344955444f,     -0.03750896453857422f,     0.13563670217990875f,     0.14953507483005524f,     -0.04516220465302467f,     -0.3529263734817505f,     0.12829488515853882f,     -0.13253933191299438f,     0.005705100484192371f,     0.11458004266023636f,     -0.17876122891902924f,     -0.053518522530794144f, 
    -0.024230413138866425f,     -0.00706451153382659f,     -0.2825150489807129f,     0.09157083183526993f,     0.1435103416442871f,     -0.016456671059131622f,     0.0035000473726540804f,     -0.2488998919725418f,     0.12046442180871964f,     0.16996313631534576f,     0.08215628564357758f,     0.08109663426876068f,     -0.23974943161010742f,     -0.01184860896319151f,     -0.015512310899794102f,     0.03236033767461777f, 
    -0.16707921028137207f,     -0.013386822305619717f,     -0.08614518493413925f,     0.07224899530410767f,     -0.15391209721565247f,     -0.03236018121242523f,     0.034229665994644165f,     -0.00944359228014946f,     0.000635840930044651f,     -0.13610298931598663f,     -0.08387816697359085f,     0.09125139564275742f,     -0.0025934914592653513f,     0.030656900256872177f,     0.07159743458032608f,     -0.08912446349859238f, 
    0.19180448353290558f,     0.00855682697147131f,     -0.05413995310664177f,     0.22921238839626312f,     -0.25168299674987793f,     0.22657498717308044f,     0.06512822955846786f,     0.12070836871862411f,     0.0957115963101387f,     -0.17223641276359558f,     0.1877749264240265f,     -0.26490676403045654f,     0.007008715998381376f,     0.03312322869896889f,     -0.30367740988731384f,     -0.2874521017074585f, 
    0.12395509332418442f,     0.046318888664245605f,     -0.09929049015045166f,     0.016164420172572136f,     -0.1808963268995285f,     -0.00030717169283889234f,     -0.0065870508551597595f,     0.11568202078342438f,     0.11862723529338837f,     -0.34521403908729553f,     0.11716940999031067f,     -0.3176097571849823f,     0.167989581823349f,     -0.03567713871598244f,     -0.1906852126121521f,     -0.20984138548374176f, 
    -0.08862079679965973f,     0.029450831934809685f,     -0.49290671944618225f,     0.11427474766969681f,     0.20569953322410583f,     -0.059454288333654404f,     0.17067784070968628f,     -0.3208756148815155f,     0.11661010980606079f,     0.19851276278495789f,     -0.02305963635444641f,     0.2132282704114914f,     -0.27051645517349243f,     0.16545435786247253f,     0.07518202811479568f,     -0.001148888492025435f, 
    0.02045145444571972f,     -0.012994020245969296f,     -0.14822687208652496f,     -0.04156552627682686f,     0.06595802307128906f,     -0.14229311048984528f,     0.030060607939958572f,     -0.28824472427368164f,     -0.0756811797618866f,     0.09642783552408218f,     0.0010478696785867214f,     -0.05786094442009926f,     -0.10664531588554382f,     0.11088093370199203f,     0.09537874162197113f,     -0.058428626507520676f, 
    -0.10591521859169006f,     0.15340842306613922f,     -0.23835721611976624f,     0.004241177346557379f,     0.11347570270299911f,     -0.07907544076442719f,     0.12370894849300385f,     -0.28463661670684814f,     0.08435549587011337f,     0.034289922565221786f,     0.05357104539871216f,     0.002960151992738247f,     -0.25761061906814575f,     0.15250223875045776f,     0.007165502291172743f,     -0.11704922467470169f, 
    -0.1133137047290802f,     0.03047521598637104f,     -0.4373946487903595f,     0.01315909344702959f,     0.19641171395778656f,     -0.0583285316824913f,     0.09449255466461182f,     -0.1404595524072647f,     0.13582274317741394f,     0.17470204830169678f,     -0.03513896092772484f,     0.2752755284309387f,     -0.2990902364253998f,     0.08145564049482346f,     0.22482214868068695f,     0.04021987318992615f, 
    0.15004020929336548f,     0.1544301062822342f,     0.5013443231582642f,     0.038738153874874115f,     -0.043102655559778214f,     0.11332681775093079f,     0.11520984768867493f,     0.09273376315832138f,     0.04105564206838608f,     -0.1416069120168686f,     -0.03622058406472206f,     -0.15263257920742035f,     0.022407008334994316f,     0.05800454691052437f,     -0.2324276864528656f,     0.009831806644797325f, 
    0.14440280199050903f,     0.19116643071174622f,     0.09538757801055908f,     0.028043504804372787f,     -0.17294585704803467f,     0.15113508701324463f,     0.16110451519489288f,     0.2280583381652832f,     0.08706498146057129f,     -0.3349861204624176f,     0.08184327930212021f,     -0.38852256536483765f,     0.2066458761692047f,     0.20113596320152283f,     -0.1506049782037735f,     -0.22081401944160461f, 
    0.10107089579105377f,     -0.1402745395898819f,     0.02171093039214611f,     -0.16944743692874908f,     0.1047530472278595f,     -0.1507573276758194f,     -0.05928938463330269f,     -0.08361171931028366f,     -0.009173481725156307f,     -0.022785058245062828f,     0.032637376338243484f,     -0.09169153869152069f,     -0.03350808098912239f,     -0.029123995453119278f,     -0.047274619340896606f,     -0.12196916341781616f, 
    0.0015314354095607996f,     0.09499990940093994f,     0.10843279212713242f,     -0.2335667610168457f,     0.22670841217041016f,     0.1053021252155304f,     -0.27346596121788025f,     0.23686224222183228f,     -0.18833085894584656f,     -0.0035045852418988943f,     -0.267751544713974f,     -0.15079286694526672f,     0.13892097771167755f,     -0.2319088727235794f,     0.12841349840164185f,     0.3247831463813782f, 
    -0.011409512721002102f,     -0.04718655347824097f,     -0.2959604561328888f,     -0.027529582381248474f,     0.07183326780796051f,     -0.21122616529464722f,     0.12400531768798828f,     -0.13795709609985352f,     0.04165232926607132f,     0.05465786159038544f,     0.10531648248434067f,     0.18164679408073425f,     -0.18200720846652985f,     0.015413831919431686f,     -0.044553451240062714f,     0.1013849526643753f, 
    0.05553019046783447f,     0.05333695560693741f,     0.02933359146118164f,     -0.07440640032291412f,     -0.047481339424848557f,     -0.04166242480278015f,     0.04025891795754433f,     -0.10162618011236191f,     0.00816301442682743f,     -0.09743352979421616f,     -0.11053134500980377f,     -0.08533278852701187f,     -0.07324087619781494f,     -0.14367209374904633f,     -0.10084217041730881f,     -0.09285172075033188f, 
    0.0785662904381752f,     0.015785394236445427f,     -0.054157886654138565f,     0.13538624346256256f,     -0.3227401077747345f,     0.028307193890213966f,     0.08979102224111557f,     0.004353669006377459f,     -0.04768037796020508f,     -0.24014416337013245f,     0.21454155445098877f,     -0.30531105399131775f,     -0.03722947835922241f,     0.17344816029071808f,     -0.2580711543560028f,     -0.16955646872520447f, 
    -0.012062125839293003f,     -0.11093426495790482f,     0.11173636466264725f,     -0.07241056859493256f,     0.010910389013588428f,     -0.13203541934490204f,     -0.12509524822235107f,     -0.01682387851178646f,     0.12702205777168274f,     -0.009769970551133156f,     -0.023576917126774788f,     -0.05688727647066116f,     -0.010073777288198471f,     -0.14122824370861053f,     -0.10401064902544022f,     0.02478632517158985f, 
    0.06339846551418304f,     0.04212146997451782f,     -0.03134700655937195f,     -0.10695277154445648f,     -0.09690593928098679f,     -0.03517110273241997f,     -0.1022503450512886f,     -0.12221280485391617f,     0.08483926951885223f,     0.082095667719841f,     -0.09802794456481934f,     0.0736457034945488f,     -0.0246697086840868f,     -0.10687897354364395f,     0.00903254747390747f,     -0.07426830381155014f, 
    0.15505178272724152f,     0.0041098278015851974f,     0.033353693783283234f,     0.2345820963382721f,     -0.1917422115802765f,     0.021421318873763084f,     0.230617955327034f,     0.09907868504524231f,     0.18224439024925232f,     -0.2716439664363861f,     0.043925393372774124f,     -0.2925634980201721f,     0.13672807812690735f,     -0.07336914539337158f,     -0.4194360375404358f,     -0.13368700444698334f, 
    0.07219117134809494f,     -0.011162659153342247f,     -0.19925057888031006f,     0.005352923180907965f,     0.1862284541130066f,     -0.25229135155677795f,     0.037774838507175446f,     -0.10593561083078384f,     0.0024846175219863653f,     -0.0414779856801033f,     0.04027646407485008f,     0.023631395772099495f,     0.06359422951936722f,     -0.000779524038080126f,     -0.03942030295729637f,     -0.05343933030962944f, 
    -0.06490524113178253f,     0.10200706124305725f,     -0.5833220481872559f,     -0.05470806360244751f,     0.17606134712696075f,     -0.24830138683319092f,     0.09784965962171555f,     -0.2750110626220703f,     0.12108739465475082f,     0.2029104381799698f,     0.0037898023147135973f,     0.21458852291107178f,     -0.2825762629508972f,     0.019409723579883575f,     0.12385642528533936f,     0.09511946141719818f, 
    0.07109502702951431f,     0.022232631221413612f,     -0.0008300977060571313f,     -0.2497262805700302f,     -0.0626787319779396f,     0.10015708953142166f,     -0.1812630444765091f,     0.0835140272974968f,     -0.19154483079910278f,     -0.11144997924566269f,     -0.3734114170074463f,     0.040602732449769974f,     0.20099444687366486f,     -0.018039021641016006f,     0.04094439372420311f,     -0.024024000391364098f, 
    0.029703186824917793f,     0.021581413224339485f,     -0.02103508822619915f,     -0.04187420383095741f,     0.05525392293930054f,     0.03345164284110069f,     0.06511230021715164f,     -0.056705739349126816f,     0.07420210540294647f,     0.00782401580363512f,     -0.03566434606909752f,     -0.06508295983076096f,     -0.0042114900425076485f,     0.04926470294594765f,     -0.08984463661909103f,     0.02588486298918724f, 
    -0.0023472653701901436f,     0.12006387114524841f,     -0.5348973274230957f,     -0.006909158546477556f,     0.20940788090229034f,     -0.18505144119262695f,     0.056361015886068344f,     -0.3200379014015198f,     -0.002369470661506057f,     0.1036379337310791f,     0.031135670840740204f,     0.10042927414178848f,     -0.3847164213657379f,     0.1650078147649765f,     0.20665933191776276f,     0.05433621257543564f, 
    0.23296837508678436f,     0.17744052410125732f,     -0.24860955774784088f,     0.19899378716945648f,     -0.2331179529428482f,     0.18831121921539307f,     -0.03503340482711792f,     0.1410842090845108f,     0.06995338946580887f,     -0.30366596579551697f,     0.0921182706952095f,     -0.31520384550094604f,     -0.05160515382885933f,     0.1799662560224533f,     -0.16333310306072235f,     -0.09300734847784042f, 
    -0.114723801612854f,     0.036930572241544724f,     -0.4638577103614807f,     0.10872723162174225f,     0.1280871033668518f,     -0.2932624816894531f,     0.19429911673069f,     -0.3049095571041107f,     0.015831325203180313f,     0.05349104106426239f,     0.09440317749977112f,     -0.03243909403681755f,     -0.2897977828979492f,     0.011258202604949474f,     0.18602491915225983f,     0.04340394213795662f, 
    -0.11758122593164444f,     -0.0039589968509972095f,     0.36314743757247925f,     0.050656501203775406f,     0.07571488618850708f,     -0.13331694900989532f,     -0.09489628672599792f,     -0.1442878246307373f,     -0.11057502031326294f,     0.08173250406980515f,     0.0678342953324318f,     -0.055335771292448044f,     -0.018113067373633385f,     -0.14227774739265442f,     -0.131214439868927f,     -0.15705476701259613f, 
    -0.045882269740104675f,     -0.03121478296816349f,     -0.029726672917604446f,     0.1674744337797165f,     -0.21839283406734467f,     0.03859007731080055f,     0.1866961419582367f,     0.08283615112304688f,     -0.04833662509918213f,     -0.341772198677063f,     0.17217637598514557f,     -0.205190047621727f,     -9.233110176865011e-05f,     0.20637983083724976f,     -0.3681747615337372f,     -0.026901258155703545f, 
    -0.058117177337408066f,     0.07846253365278244f,     -0.16775506734848022f,     0.11569221317768097f,     -0.205592080950737f,     -0.04160367697477341f,     0.1903364658355713f,     0.08790510892868042f,     0.09311414510011673f,     -0.1839519441127777f,     0.23778299987316132f,     -0.3248670697212219f,     0.024642739444971085f,     -0.027911536395549774f,     -0.3072233498096466f,     0.01215007808059454f, 
    0.12040293216705322f,     -0.16380675137043f,     -0.05052214488387108f,     -0.004469796549528837f,     0.06621844321489334f,     -0.1258976012468338f,     -0.03548828512430191f,     -0.11885575950145721f,     0.04725087806582451f,     0.06534846127033234f,     0.0784040093421936f,     -0.12113746255636215f,     0.07890571653842926f,     -0.06669991463422775f,     -0.08509055525064468f,     -0.05706460773944855f, 
};

static const std::vector<float> LOUT_B = {
    0.054656628519296646f,     0.0801088958978653f,     0.06959769874811172f,     -0.031043143942952156f,     0.014764629304409027f,     -0.09090998023748398f,     0.0851149782538414f,     0.12269727885723114f,     0.08462545275688171f,     -0.08628417551517487f,     0.04592522606253624f,     -0.09617982804775238f,     0.02718133106827736f,     -0.10514344274997711f,     -0.014109273441135883f,     0.09543371200561523f, 
    0.02083536610007286f,     0.05127760022878647f,     0.04260082170367241f,     0.07340339571237564f,     -0.038440167903900146f,     0.12961815297603607f,     0.06500028818845749f,     0.046722427010536194f,     0.023160958662629128f,     0.06377092748880386f,     0.10673776268959045f,     0.03467826917767525f,     0.07568461447954178f,     -0.014943143352866173f,     -0.07670464366674423f,     0.07425380498170853f, 
    -0.01363018061965704f,     0.1162298321723938f,     0.012882730923593044f,     -0.004204607103019953f,     0.07196374237537384f,     0.036486607044935226f,     0.10761237144470215f,     -0.03187112137675285f,     0.039848633110523224f,     0.1278686672449112f,     0.10928460210561752f,     0.08711468428373337f,     -0.07712329179048538f,     0.09165965020656586f,     0.07429438084363937f,     0.0009565967484377325f, 
};

static const std::vector<float> MEAN_W = {
    0.04187637194991112f,     0.1239093542098999f,     0.14515161514282227f,     0.0038652285002171993f,     -0.07883920520544052f,     -0.2311975508928299f,     -0.15707527101039886f,     -0.2331622838973999f,     0.22636811435222626f,     -0.22136487066745758f,     0.08773169666528702f,     0.2914273142814636f,     -0.1743471324443817f,     0.2570326626300812f,     -0.0308135524392128f,     0.14947175979614258f, 
    -0.11020998656749725f,     -0.23736637830734253f,     -0.0998091846704483f,     0.11486676335334778f,     0.021006982773542404f,     -0.18980003893375397f,     -0.13342231512069702f,     0.2037809044122696f,     0.07122691720724106f,     0.1312289834022522f,     0.20036724209785461f,     -0.06618555635213852f,     -0.1413220912218094f,     0.01411881297826767f,     0.4011121988296509f,     0.1188567653298378f, 
    -0.00025625646230764687f,     -0.22176913917064667f,     -0.0052839526906609535f,     -0.00064886111067608f,     -0.19911377131938934f,     0.03126366809010506f,     0.2147471010684967f,     0.270753413438797f,     0.001784701133146882f,     0.2138097584247589f,     -0.18391166627407074f,     0.2285204529762268f,     -0.08053290843963623f,     -0.18691177666187286f,     -0.18344856798648834f,     -0.00222193356603384f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
