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
    -0.6296991109848022f,
    -0.023392774164676666f,
    1.0f,
    -0.14682748913764954f,
    0.09352710843086243f,
    -0.8012288212776184f,
    -0.5044221878051758f,
    0.012354737147688866f,
    0.5258616805076599f,
    0.4994228184223175f,
    -0.4092680513858795f,
    -0.4256053864955902f,
    0.5456839799880981f,
    -0.4491751790046692f,
    0.5250057578086853f,
    0.4399946630001068f,
    -0.4637451469898224f,
    -0.4604331851005554f,
    -0.5501204133033752f,
    1.0f,
    0.04521411657333374f,
    -0.0031701859552413225f,
    1.0f,
    0.07052581757307053f,
    -0.5391337275505066f,
    -0.7426416277885437f,
    0.007521777879446745f,
    0.47903621196746826f,
    0.48377299308776855f,
    -0.4633951783180237f,
    -0.4299537241458893f,
    0.5013092160224915f,
    -0.494456022977829f,
    0.5031411051750183f,
    -0.3852218985557556f,
    0.5305620431900024f,
    -0.3654656410217285f,
    -0.4051835834980011f,
    -0.31719639897346497f,
    0.02560969814658165f,
    -0.08759193122386932f,
    -0.01556383166462183f,
    1.0f,
    -0.31366926431655884f,
    -0.3414519429206848f,
    -1.0f,
    -0.5771425366401672f,
    -0.5793062448501587f,
    0.4188459813594818f,
    0.4120124280452728f,
    0.4098787307739258f,
    -0.587121307849884f,
    0.4032543897628784f,
    -0.2513298988342285f,
    -0.28716573119163513f,
    0.6296711564064026f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.5114721655845642f, 
    0.3596433997154236f, 
    0.12209730595350266f, 
    -0.199943408370018f, 
    0.1350328028202057f, 
    0.36695608496665955f, 
    -0.2903050482273102f, 
    -0.4667914807796478f, 
    -0.25195077061653137f, 
    -0.22278179228305817f, 
    -0.061124641448259354f, 
    0.3934389650821686f, 
    0.48407477140426636f, 
    0.3131006062030792f, 
    -0.3878170847892761f, 
    -0.2564660906791687f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.045301247388124466f,
    -0.009442925453186035f,
    -0.11129208654165268f,
    0.07113753259181976f,
    -0.06204324960708618f,
    0.034138213843107224f,
    -0.14470352232456207f,
    -0.03998882323503494f,
    -0.06458725035190582f,
    0.15516406297683716f,
    -0.1408320963382721f,
    0.301755428314209f,
    -0.09154695272445679f,
    -0.07030469179153442f,
    -0.06815293431282043f,
    0.02342897467315197f,
};

static const std::vector<float> ANG_EMB_W = {
    -0.7522351741790771f, 
    1.3736990690231323f, 
    -0.19536232948303223f, 
    1.0034101009368896f, 
    -1.1526730060577393f, 
    -0.5381081700325012f, 
    0.28906184434890747f, 
    0.9101199507713318f, 
    -1.2502639293670654f, 
    -0.28777602314949036f, 
    0.4898248016834259f, 
    0.5026784539222717f, 
    -0.19754275679588318f, 
    0.6797904372215271f, 
    1.2111775875091553f, 
    -1.4821010828018188f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.057539571076631546f,
    -0.0025637298822402954f,
    -0.11785200983285904f,
    0.06766447424888611f,
    -0.06269264221191406f,
    0.05568476766347885f,
    -0.1000053882598877f,
    -0.047950007021427155f,
    -0.06127464771270752f,
    0.09354870766401291f,
    -0.11171543598175049f,
    0.20196278393268585f,
    -0.0919758528470993f,
    -0.062218159437179565f,
    -0.0665401741862297f,
    0.018083907663822174f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    -2.3400912284851074f, 
    1.0819939374923706f, 
    -1.090584635734558f, 
    1.8003425598144531f, 
    -2.236999273300171f, 
    -0.7483485341072083f, 
    0.9069077372550964f, 
    2.4850597381591797f, 
    -1.8204057216644287f, 
    -0.8087064027786255f, 
    0.6599844098091125f, 
    0.4680839478969574f, 
    -1.6630916595458984f, 
    1.5149815082550049f, 
    1.9775941371917725f, 
    -3.0090134143829346f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.027313411235809326f,
    -0.018800154328346252f,
    0.04627634584903717f,
    0.0258670412003994f,
    -0.04871038347482681f,
    0.01736636646091938f,
    0.03829670697450638f,
    -0.015173164196312428f,
    -0.03198343515396118f,
    -0.06093355268239975f,
    0.025358308106660843f,
    -0.16196152567863464f,
    -0.017620163038372993f,
    -0.01752675324678421f,
    -0.046866316348314285f,
    -0.00016035068256314844f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.4082983732223511f, 
    -0.22632519900798798f, 
    0.4727258086204529f, 
    -0.17731301486492157f, 
    0.2888299822807312f, 
    0.014305386692285538f, 
    0.06660634279251099f, 
    -0.538422167301178f, 
    -0.33710819482803345f, 
    0.05689392238855362f, 
    -0.034089069813489914f, 
    -0.5214594602584839f, 
    -0.0504457950592041f, 
    0.36037981510162354f, 
    -0.17014114558696747f, 
    -0.22975455224514008f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.04507969319820404f,
    -0.006442906800657511f,
    0.10183823853731155f,
    -0.06687014549970627f,
    0.064917653799057f,
    -0.026740102097392082f,
    0.15222418308258057f,
    0.04122240096330643f,
    0.0629153773188591f,
    -0.15442433953285217f,
    0.1357852965593338f,
    -0.2969367504119873f,
    0.08440729230642319f,
    0.067340187728405f,
    0.06439758092164993f,
    -0.021119078621268272f,
};

static const std::vector<float> GCN0_W = {
    -0.16788151860237122f,     0.32583388686180115f,     -0.0799405574798584f,     -0.12817849218845367f,     -0.4961615204811096f,     -0.23180411756038666f,     0.48837345838546753f,     0.34456169605255127f,     -0.6046534180641174f,     -0.31965962052345276f,     0.10025700181722641f,     -0.026367509737610817f,     -0.014627790078520775f,     0.5098982453346252f,     0.496804416179657f,     -0.15782581269741058f, 
    -0.46720999479293823f,     0.14787311851978302f,     0.5128257274627686f,     -0.42121464014053345f,     -0.1544799506664276f,     -0.44631025195121765f,     -0.045789558440446854f,     0.14792703092098236f,     -0.03735046833753586f,     -0.2739982008934021f,     0.3682914674282074f,     -0.46845054626464844f,     0.02461777627468109f,     0.09425722807645798f,     0.46674102544784546f,     0.34556692838668823f, 
    0.09397906064987183f,     -0.5920038223266602f,     0.35243546962738037f,     -0.4968266785144806f,     0.16547156870365143f,     0.1359449028968811f,     0.10656477510929108f,     -0.3012846112251282f,     0.846757173538208f,     -0.35354748368263245f,     -0.14057229459285736f,     -0.039316412061452866f,     -0.10215526819229126f,     -0.2624661922454834f,     -0.44434747099876404f,     0.49556708335876465f, 
    -0.5515755414962769f,     -0.27919238805770874f,     0.24164173007011414f,     -0.10452154278755188f,     0.09279954433441162f,     -0.2526892125606537f,     0.38884103298187256f,     0.37512704730033875f,     0.23448902368545532f,     0.026867995038628578f,     -0.08625445514917374f,     -0.3813386857509613f,     0.16686013340950012f,     0.16293708980083466f,     -0.25742289423942566f,     -0.37876272201538086f, 
    0.2635316252708435f,     0.12199579179286957f,     0.47314679622650146f,     -0.30246567726135254f,     0.5146125555038452f,     0.3292046785354614f,     0.39739736914634705f,     -0.6060082316398621f,     0.48613566160202026f,     -0.13374142348766327f,     0.35839807987213135f,     0.014827198348939419f,     0.09069665521383286f,     -0.22803516685962677f,     -0.0814736932516098f,     1.000171184539795f, 
    0.22189371287822723f,     -0.12437297403812408f,     0.2405308037996292f,     0.2048748880624771f,     0.3056517243385315f,     0.34939390420913696f,     0.05598802492022514f,     -0.12068583071231842f,     -0.1694810837507248f,     -0.41936028003692627f,     0.5082306265830994f,     -0.22485746443271637f,     8.069044270087034e-06f,     -0.0688665434718132f,     0.20351308584213257f,     -0.38316473364830017f, 
    0.42025163769721985f,     -0.13066039979457855f,     0.5572579503059387f,     -0.2092411369085312f,     0.2539922893047333f,     -0.341389536857605f,     0.4396322965621948f,     -0.31420132517814636f,     0.4493750333786011f,     -0.01662067137658596f,     0.509935200214386f,     -0.27411723136901855f,     0.6692290306091309f,     0.3411155343055725f,     0.27804136276245117f,     -0.030612368136644363f, 
    -0.2506580650806427f,     0.3796450197696686f,     -0.24667590856552124f,     0.6575959920883179f,     -0.2860497534275055f,     0.11008965969085693f,     0.24631038308143616f,     0.22707676887512207f,     -0.41192105412483215f,     -0.303323894739151f,     0.5904353260993958f,     -0.35821253061294556f,     -0.005697913933545351f,     0.5618399381637573f,     0.7579731345176697f,     -0.9963204860687256f, 
    -0.03670210391283035f,     0.29224100708961487f,     0.5383480191230774f,     -0.012090293690562248f,     -0.05800226703286171f,     -0.5748717784881592f,     -0.015200139954686165f,     0.16030633449554443f,     0.0650312677025795f,     -0.39498916268348694f,     0.335877388715744f,     -0.5749267339706421f,     0.4947280287742615f,     0.10694918036460876f,     -0.43876543641090393f,     0.17204518616199493f, 
    0.4591525197029114f,     0.2072206288576126f,     -0.079416923224926f,     -0.3866542875766754f,     0.42934197187423706f,     0.29168134927749634f,     0.28541257977485657f,     -0.20559147000312805f,     0.32068607211112976f,     0.31442931294441223f,     0.15480905771255493f,     -0.17171451449394226f,     0.1866498440504074f,     0.1947147399187088f,     -0.35130077600479126f,     0.31285417079925537f, 
    -0.3096107542514801f,     -0.24032732844352722f,     0.1442696750164032f,     -0.31070834398269653f,     0.13798226416110992f,     -0.40582752227783203f,     0.03399759158492088f,     0.6449041962623596f,     0.2560810148715973f,     -0.31996259093284607f,     0.09966189414262772f,     0.2411387413740158f,     0.21573005616664886f,     -0.15151946246623993f,     0.13027329742908478f,     -0.4375465214252472f, 
    -0.017291581258177757f,     -0.4795703589916229f,     -0.06804109364748001f,     -0.4155658781528473f,     0.3647406995296478f,     -0.10928370803594589f,     0.04228756204247475f,     -0.3510056436061859f,     0.48788732290267944f,     -0.06951113045215607f,     -0.3139522671699524f,     -0.29438889026641846f,     0.5342162847518921f,     0.09954774379730225f,     -0.06399620324373245f,     1.1013458967208862f, 
    -0.06547436863183975f,     0.6406791806221008f,     0.16099926829338074f,     0.08453547954559326f,     -0.43511244654655457f,     0.08749337494373322f,     0.508366584777832f,     0.3411835730075836f,     -0.5637391209602356f,     -0.1831466555595398f,     0.38281428813934326f,     -0.19440092146396637f,     -0.1430119425058365f,     0.03292166441679001f,     0.24753792583942413f,     -0.952484130859375f, 
    0.05743736773729324f,     0.3276607096195221f,     0.49409055709838867f,     0.14618638157844543f,     0.13149526715278625f,     0.22303566336631775f,     0.41521745920181274f,     0.08813077956438065f,     0.22263288497924805f,     0.1677217036485672f,     -0.023831602185964584f,     -0.11401458829641342f,     0.3295341730117798f,     0.2577679753303528f,     -0.030737241730093956f,     0.4922056496143341f, 
    0.5787466764450073f,     -0.09868171066045761f,     0.21461404860019684f,     -0.01841714233160019f,     0.10807441920042038f,     0.1327800750732422f,     -0.11726034432649612f,     -0.7621704339981079f,     0.44106173515319824f,     -0.3439045250415802f,     0.2139812707901001f,     -0.28028613328933716f,     0.20078040659427643f,     -0.3127118945121765f,     -0.14502781629562378f,     0.33135154843330383f, 
    0.14683109521865845f,     0.17311184108257294f,     -0.3900127708911896f,     0.6824833750724792f,     -0.4551304578781128f,     0.0050683943554759026f,     0.12030457705259323f,     -0.4219939112663269f,     -0.5680649876594543f,     0.3262229263782501f,     -0.07275635004043579f,     0.4336593449115753f,     0.19565421342849731f,     -0.2884550094604492f,     0.1001681387424469f,     -0.3004251718521118f, 
};

static const std::vector<float> GCN0_B = {
    0.046000562608242035f,
    0.2373708337545395f,
    0.09363487362861633f,
    0.08699200302362442f,
    0.052823081612586975f,
    0.04879378899931908f,
    0.19777897000312805f,
    0.08778566122055054f,
    0.18001213669776917f,
    0.07170998305082321f,
    0.11156482249498367f,
    0.03440114110708237f,
    0.0919436365365982f,
    0.10862492769956589f,
    0.0346585288643837f,
    -0.16094213724136353f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.10545878857374191f,     -0.052940987050533295f,     0.07802353799343109f,     0.015557237900793552f,     0.16718225181102753f,     -0.04836403951048851f,     0.062220003455877304f,     -0.03839436173439026f,     0.04875800386071205f,     0.16649456322193146f,     0.028232980519533157f,     0.03793355077505112f,     -0.016463924199342728f,     -0.03748315945267677f,     -0.002108398824930191f,     0.03416162729263306f, 
    -0.2632204294204712f,     0.09519853442907333f,     -0.038805410265922546f,     0.11393975466489792f,     0.1849503517150879f,     0.08836405724287033f,     0.0028219593223184347f,     -0.49018391966819763f,     -0.1458502560853958f,     0.11531859636306763f,     0.04097933694720268f,     -0.008505702950060368f,     -0.5269715785980225f,     0.04048185423016548f,     0.1561005562543869f,     0.08258865773677826f, 
    -0.1412041336297989f,     -0.07459193468093872f,     0.11080645769834518f,     0.05227190628647804f,     0.1177365705370903f,     -0.06019210070371628f,     -0.149674192070961f,     0.032948654145002365f,     -0.13529719412326813f,     -0.008209492079913616f,     -0.21760505437850952f,     0.08236414194107056f,     0.06012585386633873f,     0.027391206473112106f,     -0.12703919410705566f,     -0.04922132194042206f, 
    -0.0652337372303009f,     0.027787689119577408f,     0.0005613536341115832f,     -0.11801777780056f,     -0.003950712736696005f,     -0.04171556606888771f,     0.005560137797147036f,     -0.06904727965593338f,     0.020446866750717163f,     -0.058197271078825f,     -0.04349403828382492f,     -0.12174907326698303f,     0.07758578658103943f,     0.06092795357108116f,     -0.14714796841144562f,     -0.03395441547036171f, 
    0.11508627980947495f,     0.08211907744407654f,     -0.404037743806839f,     0.1494847685098648f,     -0.296029269695282f,     -0.09786257147789001f,     0.001373749109916389f,     0.17231927812099457f,     0.1483020782470703f,     -0.1570451259613037f,     0.13038860261440277f,     -0.21897108852863312f,     -0.08373071998357773f,     0.0778588056564331f,     -0.14921770989894867f,     -0.11353106051683426f, 
    -0.2615843713283539f,     -0.23484380543231964f,     0.14191830158233643f,     -0.16650111973285675f,     0.10114531964063644f,     -0.1295628696680069f,     -0.07242534309625626f,     -0.10205502063035965f,     -0.30666694045066833f,     -0.010472303256392479f,     -0.02927537076175213f,     0.12635497748851776f,     0.02062739059329033f,     0.0018914889078587294f,     0.05997181311249733f,     -0.08677886426448822f, 
    0.1784050315618515f,     0.15534886717796326f,     -0.25767961144447327f,     0.0934540405869484f,     -0.19148044288158417f,     0.07304788380861282f,     0.15630418062210083f,     0.0070592183619737625f,     0.07948293536901474f,     -0.20183876156806946f,     0.08657155185937881f,     -0.25827646255493164f,     0.19004133343696594f,     0.14575375616550446f,     -0.040899913758039474f,     -0.08568912744522095f, 
    0.04688669741153717f,     0.24035559594631195f,     -0.39494043588638306f,     0.09626632183790207f,     -0.09518984705209732f,     0.03219094127416611f,     0.09613622725009918f,     0.278279185295105f,     0.10508657991886139f,     -0.02383602038025856f,     0.20107406377792358f,     -0.22767524421215057f,     0.15047287940979004f,     0.01752329058945179f,     -0.21826525032520294f,     -0.1483260691165924f, 
    -0.24113723635673523f,     0.16551554203033447f,     -0.00400633504614234f,     -0.005733138415962458f,     0.21780170500278473f,     0.11203956604003906f,     0.1591016799211502f,     -0.3646181523799896f,     0.17978546023368835f,     0.24356576800346375f,     0.06903126090765f,     0.24389971792697906f,     -0.2334548532962799f,     0.1011141911149025f,     0.2278759926557541f,     -0.0532970204949379f, 
    -0.28554806113243103f,     -0.12958134710788727f,     0.12499023228883743f,     -0.12410534173250198f,     0.01858227699995041f,     -0.03294597193598747f,     -0.15613475441932678f,     -0.06633821129798889f,     -0.1488230675458908f,     0.03659835085272789f,     -0.11007672548294067f,     0.14119988679885864f,     0.11876692622900009f,     -0.09633677452802658f,     0.057387374341487885f,     -0.09346869587898254f, 
    -0.2250184863805771f,     0.07852162420749664f,     0.11538802832365036f,     0.1086234450340271f,     -0.015307036228477955f,     0.06931837648153305f,     0.05166393518447876f,     -0.35602501034736633f,     0.1263764351606369f,     0.12594515085220337f,     0.11668027192354202f,     0.0032252331729978323f,     -0.1539420336484909f,     0.15970687568187714f,     0.182245135307312f,     -0.09327277541160583f, 
    0.08919048309326172f,     -0.015225671231746674f,     -0.0629543662071228f,     -0.28311336040496826f,     -0.20392510294914246f,     -0.032769687473773956f,     -0.23222434520721436f,     0.07624784111976624f,     -0.019685082137584686f,     -0.1675916463136673f,     -0.08419667184352875f,     0.00035223158192820847f,     0.08604726940393448f,     -0.2409757375717163f,     -0.09554237127304077f,     0.21638548374176025f, 
    0.1763100028038025f,     -0.06025279685854912f,     -0.25089138746261597f,     0.13556227087974548f,     -0.21977117657661438f,     0.1989896148443222f,     0.17987678945064545f,     0.18835720419883728f,     0.1309734731912613f,     -0.10301156342029572f,     0.10003006458282471f,     -0.10428589582443237f,     0.15557803213596344f,     0.0035905963741242886f,     -0.10381808876991272f,     -0.0707157701253891f, 
    0.0165022611618042f,     -0.03938429802656174f,     0.03876788169145584f,     -0.13298845291137695f,     -0.2582527697086334f,     -0.017756659537553787f,     -0.2283741682767868f,     0.11045856028795242f,     -0.03240276128053665f,     -0.022227859124541283f,     -0.25142741203308105f,     0.07707162946462631f,     0.11505776643753052f,     -0.2075709104537964f,     -0.12979601323604584f,     0.10565686225891113f, 
    -0.09100624173879623f,     -0.05893764644861221f,     0.06891997158527374f,     0.11855339258909225f,     0.06701352447271347f,     0.07549695670604706f,     -0.06571393460035324f,     -0.11505605280399323f,     -0.10447873175144196f,     -0.118526890873909f,     -0.0363038145005703f,     -0.14463166892528534f,     0.1132742390036583f,     -0.09393465518951416f,     -0.008205006830394268f,     -0.06928147375583649f, 
    -0.026017529889941216f,     0.054252445697784424f,     -0.030317915603518486f,     -0.03355371206998825f,     0.18267236649990082f,     -0.010551867075264454f,     0.05480371043086052f,     -0.40498319268226624f,     0.08844063431024551f,     0.1687869280576706f,     0.004001341760158539f,     0.2600744366645813f,     -0.2807982563972473f,     0.13290491700172424f,     0.21366052329540253f,     0.09232057631015778f, 
    0.25366660952568054f,     -0.023264706134796143f,     -0.44834691286087036f,     0.12877191603183746f,     -0.13976465165615082f,     0.22366513311862946f,     0.008181964047253132f,     0.1293545514345169f,     0.11409949511289597f,     -0.09215830266475677f,     0.18101954460144043f,     -0.3254496157169342f,     0.12999007105827332f,     0.09253725409507751f,     -0.20847122371196747f,     -0.1759404093027115f, 
    0.19314996898174286f,     -0.10379327088594437f,     0.17229431867599487f,     0.0020436265040189028f,     -0.16606484353542328f,     -0.029079291969537735f,     -0.0653376504778862f,     0.19609923660755157f,     -0.03661848604679108f,     -0.17236679792404175f,     -0.10278543084859848f,     0.046383269131183624f,     0.10509014129638672f,     -0.010709543712437153f,     -0.008190407417714596f,     -0.16856279969215393f, 
    -0.2130308896303177f,     0.014001927338540554f,     0.0667819157242775f,     -0.058436498045921326f,     0.05280838534235954f,     -0.11539628356695175f,     0.09765558689832687f,     -0.13534753024578094f,     -0.10451972484588623f,     -0.019674839451909065f,     0.019580502063035965f,     0.21059535443782806f,     -0.20799165964126587f,     0.07443393766880035f,     0.17264436185359955f,     0.30121108889579773f, 
    -0.1557648777961731f,     -0.010561947710812092f,     -0.03959694877266884f,     0.11236413568258286f,     0.1708589643239975f,     0.1365179717540741f,     0.0123660983517766f,     -0.3621005117893219f,     0.12624193727970123f,     0.20487341284751892f,     0.07826592773199081f,     0.12295807152986526f,     -0.33280277252197266f,     -0.01655932143330574f,     0.020071078091859818f,     0.05323027819395065f, 
    -0.16886088252067566f,     -0.0021390102338045835f,     -0.05391518771648407f,     0.0849047303199768f,     -0.13624532520771027f,     -0.06828925013542175f,     0.04578010365366936f,     -0.01714053563773632f,     -0.027332700788974762f,     -0.15463481843471527f,     -0.07698320597410202f,     0.11058185994625092f,     -0.022238627076148987f,     0.021499497815966606f,     0.04828562214970589f,     -0.09166976809501648f, 
    0.24149909615516663f,     0.029790082946419716f,     -0.5449641942977905f,     0.2559243142604828f,     -0.26559963822364807f,     0.2701035141944885f,     0.056856658309698105f,     0.22493776679039001f,     0.07693449407815933f,     -0.11012571305036545f,     0.21089300513267517f,     -0.2778022289276123f,     -0.0025629489682614803f,     0.016613870859146118f,     -0.3306088447570801f,     -0.3805636465549469f, 
    0.13301724195480347f,     0.050114620476961136f,     -0.47099944949150085f,     0.0425596684217453f,     -0.21140441298484802f,     -0.013375842943787575f,     -0.019250767305493355f,     0.131772980093956f,     0.1008995845913887f,     -0.3225417137145996f,     0.12972232699394226f,     -0.3282680809497833f,     0.14595800638198853f,     -0.057482410222291946f,     -0.25501132011413574f,     -0.2316327691078186f, 
    -0.15640263259410858f,     0.05155206844210625f,     0.04850387200713158f,     0.09884152561426163f,     0.25682878494262695f,     0.1676349639892578f,     0.19814734160900116f,     -0.3946993947029114f,     0.17132599651813507f,     0.20447134971618652f,     -0.023321937769651413f,     0.23891082406044006f,     -0.2659766972064972f,     0.20004676282405853f,     0.10990855097770691f,     0.1428677886724472f, 
    -0.027440721169114113f,     0.036222632974386215f,     -0.03727218881249428f,     0.024655338376760483f,     0.10114272683858871f,     0.0645829364657402f,     0.08147672563791275f,     -0.3218643367290497f,     -0.027256401255726814f,     0.14330103993415833f,     0.036875322461128235f,     -0.007321453653275967f,     -0.13847364485263824f,     0.13831572234630585f,     0.14817285537719727f,     -0.025120818987488747f, 
    -0.21244551241397858f,     0.19185931980609894f,     0.00920106004923582f,     0.03678758814930916f,     0.16029220819473267f,     0.13408450782299042f,     0.1636418104171753f,     -0.39928126335144043f,     0.11763610690832138f,     0.0737994909286499f,     0.07228043675422668f,     0.059955935925245285f,     -0.3016533851623535f,     0.17731352150440216f,     0.07421761751174927f,     -0.032924581319093704f, 
    -0.17036694288253784f,     0.06041554734110832f,     -0.0064366827718913555f,     -0.004248442593961954f,     0.20111611485481262f,     0.1528090387582779f,     0.12953819334506989f,     -0.22034290432929993f,     0.20131562650203705f,     0.14987027645111084f,     -0.0347374752163887f,     0.2619774341583252f,     -0.29563063383102417f,     0.11657309532165527f,     0.2249159961938858f,     0.20588916540145874f, 
    0.07810649275779724f,     0.1089700385928154f,     0.15463897585868835f,     0.027239514514803886f,     0.06980577856302261f,     0.017277982085943222f,     0.08588963747024536f,     -0.00830966979265213f,     -0.024525806307792664f,     -0.00859055109322071f,     -0.10229621827602386f,     -0.012306011281907558f,     -0.07758656144142151f,     0.006962953135371208f,     -0.09989532828330994f,     0.07543523609638214f, 
    0.1722348928451538f,     0.19507649540901184f,     -0.30771782994270325f,     0.04123378172516823f,     -0.2054957151412964f,     0.15093445777893066f,     0.13701726496219635f,     0.2800849378108978f,     0.0560193732380867f,     -0.3013109862804413f,     0.09319010376930237f,     -0.42114871740341187f,     0.1701229363679886f,     0.1680976003408432f,     -0.18763472139835358f,     -0.26240500807762146f, 
    0.10147039592266083f,     -0.12985461950302124f,     0.02167169563472271f,     -0.16367247700691223f,     0.10208311676979065f,     -0.15469282865524292f,     -0.06967811286449432f,     -0.08482761681079865f,     -0.01383128110319376f,     -0.023232217878103256f,     0.013312901370227337f,     -0.08199220895767212f,     -0.03586657717823982f,     -0.030843045562505722f,     -0.04968539625406265f,     -0.12153488397598267f, 
    -0.08112877607345581f,     0.07867322117090225f,     0.05536196753382683f,     -0.13838768005371094f,     0.21931785345077515f,     -0.02117137797176838f,     -0.15716546773910522f,     0.14841249585151672f,     -0.1738024204969406f,     -0.06645067036151886f,     -0.1437191367149353f,     -0.23685665428638458f,     0.05490448698401451f,     -0.13353079557418823f,     0.08686457574367523f,     0.2594522535800934f, 
    -0.19354601204395294f,     -0.06902056932449341f,     -0.10274164378643036f,     -0.02865118719637394f,     0.09845782816410065f,     -0.08574425429105759f,     0.11549349874258041f,     -0.27981510758399963f,     0.024577906355261803f,     0.09533410519361496f,     0.07113240659236908f,     0.23053370416164398f,     -0.3016657531261444f,     -0.009988435544073582f,     -0.0040292562916874886f,     0.18984684348106384f, 
    0.054849773645401f,     0.045680932700634f,     0.02933359146118164f,     -0.09595110267400742f,     -0.06589881330728531f,     -0.008327389135956764f,     0.03253723680973053f,     -0.0818759948015213f,     0.024562036618590355f,     -0.12363786995410919f,     -0.1172880306839943f,     -0.09128601104021072f,     -0.03394876420497894f,     -0.151152104139328f,     -0.12002451717853546f,     -0.08710264414548874f, 
    0.11824864149093628f,     0.027862608432769775f,     -0.4245586097240448f,     0.10410990566015244f,     -0.3056824505329132f,     0.041507527232170105f,     0.09529903531074524f,     0.05725295841693878f,     -0.019296910613775253f,     -0.17039979994297028f,     0.2119680643081665f,     -0.3275114595890045f,     -0.011664697900414467f,     0.18174239993095398f,     -0.26761335134506226f,     -0.31617075204849243f, 
    -0.011504324153065681f,     -0.11702583730220795f,     0.10907675325870514f,     -0.07394226640462875f,     0.011821520514786243f,     -0.13236002624034882f,     -0.1244794949889183f,     -0.016920970752835274f,     0.1300889104604721f,     -0.009412653744220734f,     -0.022863510996103287f,     -0.06264406442642212f,     -0.009769205003976822f,     -0.13809192180633545f,     -0.10476034134626389f,     0.019475484266877174f, 
    0.06269722431898117f,     0.04091176390647888f,     -0.03159172832965851f,     -0.1031700149178505f,     -0.09749919921159744f,     -0.032582394778728485f,     -0.10311008989810944f,     -0.12135493010282516f,     0.0843454971909523f,     0.08136112242937088f,     -0.09673681855201721f,     0.0725083276629448f,     -0.024449950084090233f,     -0.10751441866159439f,     0.008279277011752129f,     -0.07562913000583649f, 
    0.1555829793214798f,     0.021908767521381378f,     -0.3776496648788452f,     0.21649986505508423f,     -0.13753414154052734f,     0.07283419370651245f,     0.24226894974708557f,     0.1256454437971115f,     0.21272127330303192f,     -0.2073417454957962f,     0.01366391684859991f,     -0.27886417508125305f,     0.11448711901903152f,     -0.0643351823091507f,     -0.3795284330844879f,     -0.4967802166938782f, 
    -0.01159614510834217f,     0.06953557580709457f,     -0.03812410309910774f,     0.09780123084783554f,     0.2006596326828003f,     -0.09820637851953506f,     0.1097201406955719f,     -0.1733965277671814f,     0.0675019696354866f,     0.004910895600914955f,     0.12047961354255676f,     0.05446602404117584f,     -0.03732066601514816f,     0.04642972722649574f,     -0.02894761599600315f,     0.125719353556633f, 
    -0.20086683332920074f,     0.17685888707637787f,     -0.18655410408973694f,     0.000982407364062965f,     0.19871671497821808f,     0.03407201170921326f,     0.1436479091644287f,     -0.3814433515071869f,     0.18984918296337128f,     0.2232847958803177f,     0.06969144940376282f,     0.2397136390209198f,     -0.4248918294906616f,     0.06130306050181389f,     0.11549514532089233f,     -0.008253668434917927f, 
    0.07358314841985703f,     0.06919325888156891f,     -0.04998861253261566f,     -0.005147172138094902f,     -0.15047934651374817f,     0.046829935163259506f,     -0.054946377873420715f,     0.08705417066812515f,     -0.1360529214143753f,     -0.15166623890399933f,     -0.12865440547466278f,     -0.036899205297231674f,     0.19206249713897705f,     0.09026432782411575f,     0.012694359757006168f,     0.13992230594158173f, 
    -0.05813814699649811f,     0.12338479608297348f,     0.02233252488076687f,     0.06850538402795792f,     0.10813524574041367f,     0.16455619037151337f,     0.16388167440891266f,     -0.15069697797298431f,     0.16442596912384033f,     0.09353253990411758f,     0.05095689743757248f,     0.0008740262128412724f,     -0.14184118807315826f,     0.11994786560535431f,     -0.04826187342405319f,     0.2788044214248657f, 
    -0.14189371466636658f,     0.19765622913837433f,     -0.12781430780887604f,     0.048396140336990356f,     0.2167309671640396f,     0.12311630696058273f,     0.10043631494045258f,     -0.4230152368545532f,     0.07574836164712906f,     0.10980284214019775f,     0.10643377155065536f,     0.10233224183320999f,     -0.5146240592002869f,     0.20835630595684052f,     0.1833532601594925f,     -0.06293404847383499f, 
    0.24663178622722626f,     0.2004203349351883f,     -0.6756713390350342f,     0.1850835680961609f,     -0.18590514361858368f,     0.2307591289281845f,     -0.01718580536544323f,     0.16661636531352997f,     0.08886869996786118f,     -0.20107218623161316f,     0.07630178332328796f,     -0.3050292134284973f,     -0.08775071799755096f,     0.20321695506572723f,     -0.150680810213089f,     -0.4544394910335541f, 
    -0.06377024948596954f,     -0.01532799657434225f,     -0.09078329056501389f,     0.058333829045295715f,     0.011841588653624058f,     -0.11719194054603577f,     0.11679389327764511f,     -0.06686091423034668f,     -0.05258403345942497f,     -0.04464006796479225f,     0.051337581127882004f,     -0.1369611769914627f,     -0.022280370816588402f,     -0.0873333290219307f,     0.07297052443027496f,     0.06424428522586823f, 
    0.0809703916311264f,     0.20061518251895905f,     -0.3745204210281372f,     0.18134009838104248f,     -0.1139792650938034f,     0.025897933170199394f,     0.08731886744499207f,     0.04340115189552307f,     0.09149730205535889f,     -0.11384131759405136f,     0.1388310045003891f,     -0.36798349022865295f,     0.06683912128210068f,     0.017684675753116608f,     -0.41171374917030334f,     -0.5420048832893372f, 
    -0.017228489741683006f,     -0.07021436840295792f,     -0.3714398443698883f,     0.06388173252344131f,     -0.25031083822250366f,     -0.0071084462106227875f,     0.12788143754005432f,     0.10941570997238159f,     -0.078061543405056f,     -0.3479095697402954f,     0.12848277390003204f,     -0.2767690122127533f,     0.051979370415210724f,     0.15291500091552734f,     -0.41693225502967834f,     -0.032866138964891434f, 
    -0.09855034202337265f,     -0.01251152716577053f,     -0.13521885871887207f,     -0.07205434888601303f,     -0.04430651664733887f,     -0.12525725364685059f,     0.07828149944543839f,     -0.008948924951255322f,     0.017636559903621674f,     0.013114959932863712f,     0.06872782111167908f,     -0.020427703857421875f,     -0.010817649774253368f,     -0.13365405797958374f,     -0.09047479927539825f,     0.10172133147716522f, 
    0.09341223537921906f,     -0.15761229395866394f,     -0.048553500324487686f,     0.03290277346968651f,     0.0349590890109539f,     -0.12159889191389084f,     -0.022556539624929428f,     -0.12725812196731567f,     0.03564189374446869f,     0.06788913160562515f,     0.08594566583633423f,     -0.12690289318561554f,     0.04216630756855011f,     -0.07819920033216476f,     -0.10060007125139236f,     -0.06348719447851181f, 
};

static const std::vector<float> LOUT_B = {
    0.02774895541369915f,     -0.0039503867737948895f,     -0.08041256666183472f,     -0.04549729451537132f,     0.03846761956810951f,     -0.1122765988111496f,     0.10754457116127014f,     0.1557142734527588f,     0.06128637120127678f,     -0.10850219428539276f,     0.07248077541589737f,     -0.13260193169116974f,     0.05265219137072563f,     -0.1316968947649002f,     -0.011978978291153908f,     0.06348928064107895f, 
    0.02570444718003273f,     -0.07666332274675369f,     -0.007112628780305386f,     0.06282011419534683f,     -0.026963606476783752f,     0.13876011967658997f,     0.05503059923648834f,     0.06333408504724503f,     0.05571555346250534f,     0.087739959359169f,     0.1219559833407402f,     0.004086390603333712f,     0.07459565252065659f,     -0.015071489848196507f,     -0.08866468071937561f,     0.0502394400537014f, 
    -0.027037598192691803f,     0.11723630130290985f,     0.01404182892292738f,     -0.004885538015514612f,     0.06995201110839844f,     0.09151829779148102f,     0.15395836532115936f,     0.011572242714464664f,     0.09309062361717224f,     0.17365576326847076f,     0.1198272630572319f,     0.026703841984272003f,     0.08029552549123764f,     0.06111879646778107f,     -0.011285576038062572f,     -0.005851115565747023f, 
};

static const std::vector<float> MEAN_W = {
    0.02516559511423111f,     0.14050689339637756f,     -0.1118072122335434f,     -0.0067171575501561165f,     -0.07308690249919891f,     -0.2950435280799866f,     -0.12469694763422012f,     -0.23333457112312317f,     0.20021669566631317f,     -0.23323428630828857f,     0.11424089968204498f,     0.26456210017204285f,     -0.13450317084789276f,     0.22540298104286194f,     -0.02990342117846012f,     0.11037832498550415f, 
    -0.1562260091304779f,     -0.13679268956184387f,     0.09901488572359085f,     0.13759121298789978f,     0.022983266040682793f,     -0.24666394293308258f,     -0.16515986621379852f,     0.21152432262897491f,     0.08193386346101761f,     0.1692340075969696f,     0.1897096484899521f,     0.006722612772136927f,     -0.1601465493440628f,     0.002496524481102824f,     0.2612532675266266f,     0.19277659058570862f, 
    0.03248341754078865f,     -0.22146302461624146f,     -0.008933242410421371f,     0.0006198462215252221f,     -0.21639849245548248f,     0.08638548851013184f,     0.23222847282886505f,     -0.07298664003610611f,     0.11698681116104126f,     0.2366642951965332f,     -0.23454071581363678f,     -0.003325001336634159f,     -0.20122410356998444f,     -0.12249312549829483f,     0.008055079728364944f,     0.015523811802268028f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
