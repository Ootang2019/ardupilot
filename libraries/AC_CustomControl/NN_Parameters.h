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
    -0.3198982775211334f,
    0.056809306144714355f,
    1.0f,
    -0.09029915928840637f,
    0.32598647475242615f,
    -1.0f,
    -0.4868558943271637f,
    -0.05422850698232651f,
    0.4506947696208954f,
    0.4274217486381531f,
    -0.5345067977905273f,
    -0.5808359980583191f,
    0.38792452216148376f,
    -0.6012941002845764f,
    0.38796305656433105f,
    0.5947836637496948f,
    -0.2571447193622589f,
    -0.3522559404373169f,
    -0.1767772138118744f,
    1.0f,
    0.04198053106665611f,
    -0.3240125775337219f,
    1.0f,
    -0.19229726493358612f,
    -0.35368314385414124f,
    -1.0f,
    0.05957605689764023f,
    0.4506596624851227f,
    0.4273078441619873f,
    -0.4809119403362274f,
    -0.517020046710968f,
    0.3800099492073059f,
    -0.5900444984436035f,
    0.3946678936481476f,
    -0.21677273511886597f,
    0.6230264902114868f,
    -0.27903762459754944f,
    -0.5133746266365051f,
    -0.26118093729019165f,
    0.006089067552238703f,
    -0.4048270583152771f,
    0.0663696676492691f,
    1.0f,
    -0.37263062596321106f,
    -0.6218639016151428f,
    -0.4851560592651367f,
    -0.5318318009376526f,
    -0.512822687625885f,
    0.42836877703666687f,
    0.43984630703926086f,
    0.4341740906238556f,
    -0.5605148673057556f,
    0.43724748492240906f,
    -0.2887154519557953f,
    -0.29556289315223694f,
    0.5322563052177429f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.524773359298706f, 
    0.451095849275589f, 
    0.05308537930250168f, 
    -0.12363359332084656f, 
    0.05100834369659424f, 
    0.22531047463417053f, 
    -0.2837829887866974f, 
    -0.42825552821159363f, 
    -0.2837655544281006f, 
    -0.30678948760032654f, 
    -0.1049695834517479f, 
    0.4350968599319458f, 
    0.4284227788448334f, 
    0.3583276569843292f, 
    -0.32121536135673523f, 
    -0.3001464903354645f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.03038755990564823f,
    -0.053286854177713394f,
    -0.06323901563882828f,
    0.07842949777841568f,
    -0.075213223695755f,
    -0.010259115137159824f,
    -0.08849402517080307f,
    -0.03132960572838783f,
    -0.05872480943799019f,
    0.038965895771980286f,
    -0.09778536856174469f,
    0.12438762187957764f,
    -0.09161930531263351f,
    -0.07707598060369492f,
    -0.021628737449645996f,
    0.01337680034339428f,
};

static const std::vector<float> ANG_EMB_W = {
    -0.8277996778488159f, 
    0.77260422706604f, 
    -0.5834262371063232f, 
    1.2108449935913086f, 
    -1.3726584911346436f, 
    -0.7811329364776611f, 
    0.20518375933170319f, 
    1.1854227781295776f, 
    -1.7352548837661743f, 
    -0.7370522618293762f, 
    0.7912851572036743f, 
    0.66546630859375f, 
    -0.35215336084365845f, 
    0.7969861626625061f, 
    1.6278711557388306f, 
    -2.038205146789551f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.03409072756767273f,
    -0.03584204986691475f,
    -0.09537969529628754f,
    0.0769299864768982f,
    -0.08794665336608887f,
    0.01418898906558752f,
    -0.09905210137367249f,
    -0.03215678408741951f,
    -0.05715399608016014f,
    0.03121856600046158f,
    -0.13500827550888062f,
    0.17204070091247559f,
    -0.09131303429603577f,
    -0.10208165645599365f,
    -0.029101457446813583f,
    0.009605436585843563f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    -2.6843481063842773f, 
    -0.8520429730415344f, 
    -1.338221788406372f, 
    1.4765961170196533f, 
    -2.449047565460205f, 
    -1.1350281238555908f, 
    1.030150294303894f, 
    2.803105115890503f, 
    -2.0069186687469482f, 
    -2.202857494354248f, 
    1.0331337451934814f, 
    0.6382776498794556f, 
    -1.902022123336792f, 
    1.0981764793395996f, 
    2.5211668014526367f, 
    -3.6492536067962646f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.0404144786298275f,
    -0.04481789469718933f,
    0.005794197786599398f,
    0.003920942544937134f,
    -0.036240458488464355f,
    0.02030506171286106f,
    -0.03561997786164284f,
    -0.036164358258247375f,
    -0.02593129314482212f,
    0.02131841890513897f,
    0.015026140958070755f,
    -0.012800089083611965f,
    -0.04601876065135002f,
    -0.02715645544230938f,
    -0.03693335875868797f,
    0.018671169877052307f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.409063458442688f, 
    -0.4087969958782196f, 
    0.3483069837093353f, 
    -0.11621269583702087f, 
    0.2463580220937729f, 
    0.011373135261237621f, 
    0.23217985033988953f, 
    -0.5590469837188721f, 
    -0.36394649744033813f, 
    -0.015164000913500786f, 
    -0.008586622774600983f, 
    -0.4061773419380188f, 
    -0.024327358230948448f, 
    0.35120272636413574f, 
    -0.16896945238113403f, 
    -0.2547960579395294f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.012682987377047539f,
    0.018866218626499176f,
    0.03719976916909218f,
    -0.04610004648566246f,
    0.07893045246601105f,
    0.04131494462490082f,
    -0.01700783334672451f,
    0.015343171544373035f,
    0.05874353647232056f,
    0.0053223734721541405f,
    0.03804456815123558f,
    -0.0732928141951561f,
    0.05982691049575806f,
    0.004722672514617443f,
    0.010478687472641468f,
    0.0011503099231049418f,
};

static const std::vector<float> GCN0_W = {
    -0.2877200245857239f,     0.15926653146743774f,     -0.06496620178222656f,     -0.14172354340553284f,     -0.5116748809814453f,     -0.1880064308643341f,     0.49071574211120605f,     0.4313051998615265f,     -0.7174832820892334f,     -0.44854775071144104f,     0.12703678011894226f,     -0.02080088108778f,     0.021663116291165352f,     0.5023233294487f,     0.6691410541534424f,     -0.7064197063446045f, 
    -0.4181845486164093f,     0.0819140076637268f,     0.28374379873275757f,     -0.4212483763694763f,     -0.1570105254650116f,     -0.17285530269145966f,     -0.26253965497016907f,     0.09269437193870544f,     -0.03305257484316826f,     -0.1349981427192688f,     0.15111663937568665f,     -0.3493889272212982f,     -0.07328546047210693f,     -0.028013011440634727f,     0.3582337498664856f,     0.3284311592578888f, 
    0.4417896866798401f,     -0.6473878622055054f,     0.2953670024871826f,     -0.413629949092865f,     0.04670969396829605f,     0.5534643530845642f,     -0.30669966340065f,     -0.4535503089427948f,     0.7672614455223083f,     0.3636418282985687f,     -1.6253836154937744f,     0.16000281274318695f,     -0.18330132961273193f,     -1.2350057363510132f,     -0.700998067855835f,     0.5448889136314392f, 
    -0.6889122128486633f,     -0.4833204746246338f,     0.2272639125585556f,     -0.0771082192659378f,     0.025010600686073303f,     -0.23673006892204285f,     0.43134406208992004f,     0.46716630458831787f,     0.14943210780620575f,     -0.05409012734889984f,     -0.07736576348543167f,     -0.4787876307964325f,     0.11786920577287674f,     0.13306449353694916f,     -0.15597723424434662f,     -0.48141127824783325f, 
    0.27661624550819397f,     0.1627682000398636f,     0.5090387463569641f,     -0.32456716895103455f,     0.59245365858078f,     0.20751117169857025f,     0.46144217252731323f,     -0.6899331212043762f,     0.6036010980606079f,     0.005322271026670933f,     0.40020284056663513f,     -0.03978263586759567f,     0.05841248109936714f,     -0.14489136636257172f,     -0.3562816381454468f,     1.4268913269042969f, 
    0.3574709892272949f,     -0.1710042506456375f,     0.10168924182653427f,     0.2326505482196808f,     0.3378617465496063f,     0.36417242884635925f,     -0.05824129655957222f,     -0.17124909162521362f,     -0.13696777820587158f,     -0.3429313004016876f,     0.44947758316993713f,     -0.09207114577293396f,     -0.050762567669153214f,     -0.15974968671798706f,     0.15296947956085205f,     -0.22613896429538727f, 
    0.2688025236129761f,     -0.015214242972433567f,     0.43702432513237f,     -0.19313481450080872f,     0.23028136789798737f,     -0.18603675067424774f,     0.31382352113723755f,     -0.2774229943752289f,     0.3834418058395386f,     0.021247850731015205f,     0.42215481400489807f,     -0.15002305805683136f,     0.6106021404266357f,     0.3835650682449341f,     0.2772689759731293f,     -0.17332306504249573f, 
    -0.2627390921115875f,     0.12465518712997437f,     -0.2726846933364868f,     0.6304747462272644f,     -0.2768557667732239f,     0.20844592154026031f,     0.20657244324684143f,     0.2444709688425064f,     -0.47205767035484314f,     -0.41422581672668457f,     0.5880178213119507f,     -0.2979613244533539f,     0.026355521753430367f,     0.5047623515129089f,     0.9138726592063904f,     -1.5503795146942139f, 
    -0.06233896687626839f,     0.4141896665096283f,     0.3352271318435669f,     0.06856843084096909f,     -0.16593855619430542f,     -0.43042388558387756f,     -0.12844742834568024f,     0.44617965817451477f,     0.0602727010846138f,     -0.40880417823791504f,     0.34335997700691223f,     -0.4106626808643341f,     0.4106130003929138f,     0.16088174283504486f,     -0.32297539710998535f,     -0.02957107685506344f, 
    0.3404495418071747f,     0.4333342909812927f,     -0.07344557344913483f,     -0.3415857255458832f,     0.3825764060020447f,     0.2339724749326706f,     0.37500056624412537f,     0.04304063320159912f,     0.3215966820716858f,     0.19887211918830872f,     0.3093951344490051f,     -0.15307949483394623f,     0.23836055397987366f,     0.40219858288764954f,     -0.357880175113678f,     0.12926749885082245f, 
    -0.492977499961853f,     -0.5323982834815979f,     0.14728234708309174f,     -0.2866641581058502f,     0.08346004784107208f,     -0.36576560139656067f,     0.017995722591876984f,     0.7219496369361877f,     0.07191116362810135f,     -0.38287869095802307f,     0.08990323543548584f,     0.3013986349105835f,     0.14268606901168823f,     -0.12044618278741837f,     0.27573341131210327f,     -0.943645715713501f, 
    -0.025723829865455627f,     -0.37221765518188477f,     -0.006152528803795576f,     -0.4156118631362915f,     0.43292924761772156f,     -0.25572147965431213f,     0.11190076172351837f,     -0.5128772854804993f,     0.5460323691368103f,     0.07394609600305557f,     -0.24704748392105103f,     -0.31846752762794495f,     0.5101271867752075f,     0.2280663549900055f,     -0.3779224157333374f,     1.4454468488693237f, 
    -0.13110366463661194f,     0.2161072939634323f,     0.12991832196712494f,     0.03156013786792755f,     -0.42363300919532776f,     0.28841081261634827f,     0.46096211671829224f,     0.28270456194877625f,     -0.5084212422370911f,     -0.26852887868881226f,     0.3502783477306366f,     -0.20078091323375702f,     -0.22590254247188568f,     -0.06001008674502373f,     0.26546719670295715f,     -1.2507063150405884f, 
    0.1827186644077301f,     0.4420873820781708f,     0.4638921916484833f,     0.025785930454730988f,     0.34697094559669495f,     0.12388286739587784f,     0.38760605454444885f,     -0.3432345986366272f,     0.3645634353160858f,     0.3709169328212738f,     -0.0947544053196907f,     -0.1418619155883789f,     0.3019050359725952f,     0.255630761384964f,     -0.4043308198451996f,     1.4101017713546753f, 
    0.589624285697937f,     -0.016873642802238464f,     0.2696631848812103f,     -0.025625435635447502f,     0.16476070880889893f,     -0.06088712066411972f,     -0.04326562583446503f,     -0.7745450139045715f,     0.5343517065048218f,     -0.24251018464565277f,     0.27537450194358826f,     -0.31010711193084717f,     0.20173625648021698f,     -0.1924872100353241f,     -0.37414461374282837f,     0.756132185459137f, 
    -0.19412392377853394f,     0.38094305992126465f,     -1.1335047483444214f,     0.8935396671295166f,     -0.774730920791626f,     -0.2004755586385727f,     0.18680836260318756f,     -0.22226345539093018f,     -0.7886079549789429f,     -0.03005899488925934f,     0.03301715478301048f,     0.681559681892395f,     -0.10385231673717499f,     -0.12379778921604156f,     0.31401535868644714f,     -0.5535748600959778f, 
};

static const std::vector<float> GCN0_B = {
    0.04055630788207054f,
    0.16347531974315643f,
    -0.19037027657032013f,
    0.17946651577949524f,
    0.08697982132434845f,
    0.053976576775312424f,
    0.1062619760632515f,
    0.05884460732340813f,
    0.15749140083789825f,
    0.1434091329574585f,
    0.07676728814840317f,
    0.04072526469826698f,
    0.07890526205301285f,
    0.048653632402420044f,
    0.029109274968504906f,
    -0.21060442924499512f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.04082988202571869f,     0.0470082089304924f,     -0.6343656778335571f,     0.06967214494943619f,     0.1642458289861679f,     0.05237002298235893f,     0.11010856181383133f,     0.05246225371956825f,     0.1822212040424347f,     0.2290034145116806f,     0.06103939935564995f,     -0.08821909874677658f,     0.11617913097143173f,     -0.057789575308561325f,     -0.03966999426484108f,     -0.12448979169130325f, 
    -0.0855942964553833f,     0.2487366944551468f,     -0.0915939137339592f,     0.2644061744213104f,     0.28561583161354065f,     0.2539602816104889f,     0.13005264103412628f,     -0.21864187717437744f,     -0.017241351306438446f,     0.1670767068862915f,     0.15148186683654785f,     0.08105834573507309f,     -0.12918542325496674f,     0.18019884824752808f,     0.24000225961208344f,     -0.026074441149830818f, 
    -0.22239796817302704f,     0.135964035987854f,     0.26009026169776917f,     0.20391985774040222f,     0.2969669699668884f,     0.09261836856603622f,     0.026143306866288185f,     -0.16659364104270935f,     0.02922188490629196f,     0.08676014840602875f,     -0.15036465227603912f,     0.28573504090309143f,     -0.08783069998025894f,     0.24221429228782654f,     0.024826381355524063f,     -0.19555138051509857f, 
    0.06149369478225708f,     0.1678556501865387f,     -0.043588194996118546f,     0.04067116603255272f,     -0.14730654656887054f,     0.07314372807741165f,     0.12367406487464905f,     0.07448922842741013f,     0.15119393169879913f,     0.06409624963998795f,     0.15947845578193665f,     -0.31627407670021057f,     0.20370259881019592f,     -0.0009367940365336835f,     -0.33499589562416077f,     -0.03635439649224281f, 
    0.1357361078262329f,     0.12799930572509766f,     -0.22056333720684052f,     0.16407941281795502f,     -0.15207916498184204f,     -0.05848374217748642f,     0.026611682027578354f,     0.18635281920433044f,     0.22529776394367218f,     0.11217893660068512f,     0.20142512023448944f,     -0.09353053569793701f,     -0.02550448663532734f,     0.027247626334428787f,     0.08233998715877533f,     -0.006227628793567419f, 
    -0.156904935836792f,     -0.13469597697257996f,     0.20799486339092255f,     -0.1307438462972641f,     0.04533330723643303f,     -0.21766327321529388f,     -0.12227829545736313f,     -0.14212541282176971f,     -0.38928139209747314f,     -0.08184917271137238f,     -0.028738902881741524f,     0.07211996614933014f,     -0.060619596391916275f,     0.015074240043759346f,     -0.013946477323770523f,     -0.10261404514312744f, 
    0.2357402890920639f,     0.19489705562591553f,     -0.022043880075216293f,     0.12846337258815765f,     -0.20122183859348297f,     0.12397205829620361f,     0.17536561191082f,     0.07364808022975922f,     0.14670927822589874f,     0.0068253581412136555f,     0.1824667751789093f,     -0.27033933997154236f,     0.27716049551963806f,     0.03716593608260155f,     -0.025637516751885414f,     0.11143684387207031f, 
    0.11402300000190735f,     0.27323856949806213f,     -0.1453959345817566f,     0.11621410399675369f,     -0.12210415303707123f,     0.05131015181541443f,     0.11568822711706161f,     0.3502509593963623f,     0.16298194229602814f,     0.16104307770729065f,     0.2894238531589508f,     -0.2519521117210388f,     0.23867373168468475f,     -0.09580042213201523f,     -0.2369314283132553f,     0.005187423899769783f, 
    -0.2786940336227417f,     0.1505943238735199f,     0.08166671544313431f,     -0.014743315987288952f,     0.2366439551115036f,     0.09699609875679016f,     0.15872691571712494f,     -0.3206610679626465f,     0.1485275775194168f,     0.19626300036907196f,     -0.028017137199640274f,     0.28681278228759766f,     -0.10284080356359482f,     0.15322193503379822f,     0.20300021767616272f,     -0.11075988411903381f, 
    -0.2685234546661377f,     0.09119521081447601f,     0.4223904013633728f,     -0.013757392764091492f,     0.12066002935171127f,     0.06267056614160538f,     0.024654965847730637f,     -0.2860395014286041f,     -0.00954474601894617f,     0.0644913837313652f,     -0.09923237562179565f,     0.2710729241371155f,     -0.052725598216056824f,     0.09479201585054398f,     0.11931980401277542f,     -0.19037166237831116f, 
    -0.21535971760749817f,     0.12133356928825378f,     0.08297859877347946f,     0.16433724761009216f,     0.0429353229701519f,     0.1127408966422081f,     0.09524765610694885f,     -0.25930550694465637f,     0.15742649137973785f,     0.14083027839660645f,     0.10425390303134918f,     0.06001409888267517f,     0.07264921069145203f,     0.21435481309890747f,     0.22588467597961426f,     -0.11128900945186615f, 
    0.09051945805549622f,     -0.14594578742980957f,     -0.0780860185623169f,     -0.42638468742370605f,     -0.11725280433893204f,     -0.14297379553318024f,     -0.2657017111778259f,     0.1013040691614151f,     -0.04122745245695114f,     -0.5868290066719055f,     0.02957511879503727f,     0.022035792469978333f,     0.07525955885648727f,     -0.2858511805534363f,     -0.08716906607151031f,     0.32629767060279846f, 
    0.2565329074859619f,     -0.07425463199615479f,     0.1827862560749054f,     0.10974004119634628f,     -0.3145918548107147f,     0.15633991360664368f,     0.16032816469669342f,     0.2680516541004181f,     0.12271042168140411f,     0.03354446962475777f,     0.19392819702625275f,     -0.16556349396705627f,     0.23241983354091644f,     -0.19197064638137817f,     -0.1732454001903534f,     0.2723585069179535f, 
    0.055756356567144394f,     -0.010916445404291153f,     0.022920764982700348f,     -0.09911561757326126f,     -0.20052620768547058f,     0.040118101984262466f,     -0.16504785418510437f,     0.15148086845874786f,     -0.007464144844561815f,     -0.3789757192134857f,     -0.12041809409856796f,     0.056521885097026825f,     0.12922857701778412f,     -0.17100444436073303f,     -0.12228721380233765f,     0.2034614235162735f, 
    -0.08769288659095764f,     -0.10354533046483994f,     0.052050936967134476f,     0.06976926326751709f,     0.04782452434301376f,     0.04137592762708664f,     -0.07986216992139816f,     -0.11422067135572433f,     -0.12826645374298096f,     -0.1369914561510086f,     -0.0627463310956955f,     -0.15853148698806763f,     0.09054073691368103f,     -0.09381124377250671f,     -0.01301947794854641f,     -0.07003491371870041f, 
    -0.052996277809143066f,     0.05315633863210678f,     -0.04605228826403618f,     -0.019171051681041718f,     0.21348176896572113f,     -0.004502732772380114f,     0.0572667233645916f,     -0.34605297446250916f,     0.06958460062742233f,     0.1273692399263382f,     -0.07474356889724731f,     0.28894689679145813f,     -0.10118365287780762f,     0.20502910017967224f,     0.22008933126926422f,     -0.005929884966462851f, 
    0.20948836207389832f,     -0.08363882452249527f,     -0.07359520345926285f,     0.08543125540018082f,     -0.16358226537704468f,     0.1765701025724411f,     -0.034399960190057755f,     0.09391312301158905f,     0.05497831478714943f,     0.0531940795481205f,     0.20406192541122437f,     -0.31775134801864624f,     0.1435559093952179f,     -0.14512933790683746f,     -0.17004604637622833f,     -0.05608491972088814f, 
    0.22301335632801056f,     -0.04828852787613869f,     0.4697127640247345f,     0.04804961383342743f,     -0.19219346344470978f,     0.04327457770705223f,     -0.00963557232171297f,     0.2381240427494049f,     0.04680430889129639f,     -0.1743754893541336f,     0.07621462643146515f,     0.009491143748164177f,     0.13985119760036469f,     -0.006164105609059334f,     -0.01766100898385048f,     0.03830092027783394f, 
    -0.24820475280284882f,     -0.07658141106367111f,     0.33439067006111145f,     -0.1162438839673996f,     0.015832094475626945f,     -0.1969871073961258f,     0.020699942484498024f,     -0.23704040050506592f,     -0.21487443149089813f,     -0.14980775117874146f,     -0.07109150290489197f,     0.19070446491241455f,     -0.38822197914123535f,     0.1336788833141327f,     0.10311462730169296f,     0.34423041343688965f, 
    0.0019627311266958714f,     0.09507162868976593f,     -0.29590925574302673f,     0.17545165121555328f,     0.21577809751033783f,     0.18886087834835052f,     0.06963030248880386f,     -0.12345506995916367f,     0.24796698987483978f,     0.278602659702301f,     0.09200863540172577f,     0.11809001117944717f,     -0.04337729513645172f,     0.03941028192639351f,     0.03423823043704033f,     -0.15986832976341248f, 
    -0.30135852098464966f,     -0.001953746657818556f,     -0.09496799856424332f,     0.06797615438699722f,     0.000255304534221068f,     -0.04987332969903946f,     0.05507110059261322f,     -0.22340501844882965f,     0.047861989587545395f,     -0.05729518085718155f,     -0.15598182380199432f,     0.17686955630779266f,     -0.16745752096176147f,     0.1555977165699005f,     0.13839082419872284f,     -0.03456176817417145f, 
    0.24736128747463226f,     0.027967754751443863f,     -0.14033815264701843f,     0.25947922468185425f,     -0.2247798591852188f,     0.2643734812736511f,     0.0777868926525116f,     0.24096809327602386f,     0.08519835770130157f,     0.1051410511136055f,     0.2474852353334427f,     -0.21451272070407867f,     0.07115843892097473f,     -0.14505349099636078f,     -0.24298737943172455f,     -0.29003143310546875f, 
    0.159659281373024f,     0.09716133773326874f,     -0.1062837690114975f,     0.09058123081922531f,     -0.07624611258506775f,     0.05791116878390312f,     0.03860240429639816f,     0.18059058487415314f,     0.14779280126094818f,     -0.021979760378599167f,     0.21421287953853607f,     -0.1686907261610031f,     0.20994716882705688f,     -0.16141079366207123f,     -0.02243996411561966f,     -0.16364270448684692f, 
    -0.11757722496986389f,     0.023628761991858482f,     -0.029847366735339165f,     0.10155453532934189f,     0.23195256292819977f,     0.16721941530704498f,     0.15168015658855438f,     -0.3220084309577942f,     0.13520434498786926f,     0.16673322021961212f,     -0.06924645602703094f,     0.23783279955387115f,     -0.12438980489969254f,     0.20943602919578552f,     0.03376008942723274f,     0.05122238025069237f, 
    0.0909060388803482f,     -0.06854496896266937f,     0.013836726546287537f,     -0.06939249485731125f,     0.03209329769015312f,     -0.04329650476574898f,     -0.017654063180088997f,     -0.15343032777309418f,     -0.11345192790031433f,     0.03344334661960602f,     0.009250577539205551f,     -0.05859876424074173f,     0.062252335250377655f,     0.09836632758378983f,     0.07787398993968964f,     0.00494202459231019f, 
    -0.09864140301942825f,     0.22384221851825714f,     -0.145675390958786f,     0.09727760404348373f,     0.16727638244628906f,     0.16732728481292725f,     0.15800245106220245f,     -0.23733343183994293f,     0.15019045770168304f,     0.08215706050395966f,     0.10737244039773941f,     0.06466753035783768f,     -0.08647750318050385f,     0.19930076599121094f,     0.023327339440584183f,     -0.10939185321331024f, 
    -0.13412591814994812f,     0.0645093098282814f,     -0.1521582454442978f,     0.038698527961969376f,     0.19025123119354248f,     0.17514412105083466f,     0.09900068491697311f,     -0.13720855116844177f,     0.20013946294784546f,     0.1424000859260559f,     -0.05171593651175499f,     0.2599499821662903f,     -0.14635971188545227f,     0.12157783657312393f,     0.15831731259822845f,     0.08057636022567749f, 
    0.09848130494356155f,     0.27172577381134033f,     -0.0937102809548378f,     0.12226708978414536f,     0.07290271669626236f,     0.152234748005867f,     0.18661099672317505f,     0.010580210946500301f,     0.15316170454025269f,     0.085938461124897f,     0.0021089084912091494f,     -0.07456709444522858f,     0.00431561516597867f,     0.03741380572319031f,     -0.06509368866682053f,     -0.22218890488147736f, 
    0.16916996240615845f,     0.22730420529842377f,     0.07817238569259644f,     0.06930830329656601f,     -0.1064186841249466f,     0.17460110783576965f,     0.19480156898498535f,     0.2741681635379791f,     0.07733665406703949f,     -0.035188913345336914f,     0.14116309583187103f,     -0.28489646315574646f,     0.21113865077495575f,     0.04529949277639389f,     -0.01714305393397808f,     -0.18338266015052795f, 
    0.10235917568206787f,     -0.13075803220272064f,     0.021841347217559814f,     -0.14174966514110565f,     0.10453824698925018f,     -0.15416915714740753f,     -0.0660102590918541f,     -0.08516274392604828f,     -0.011376308277249336f,     -0.02146867848932743f,     0.01947186142206192f,     -0.07754278928041458f,     -0.034855447709560394f,     -0.029019901528954506f,     -0.047562114894390106f,     -0.12202456593513489f, 
    -0.14121957123279572f,     0.042183589190244675f,     0.09738311171531677f,     -0.09929782152175903f,     0.1034175381064415f,     -0.021594736725091934f,     -0.16966551542282104f,     0.10128502547740936f,     -0.16306285560131073f,     -0.11208084970712662f,     -0.08364655077457428f,     -0.14182156324386597f,     0.007972712628543377f,     -0.06011927127838135f,     -0.03888342157006264f,     0.026005959138274193f, 
    -0.16202868521213531f,     -0.1467527598142624f,     -0.17749464511871338f,     -0.08600098639726639f,     0.04585926607251167f,     -0.15157382190227509f,     0.023913802579045296f,     -0.23938900232315063f,     -0.05903533101081848f,     -0.042883824557065964f,     -0.09913987666368484f,     0.18336108326911926f,     -0.26031744480133057f,     0.003116386476904154f,     -0.08247528225183487f,     0.34318557381629944f, 
    0.07010067999362946f,     0.06361453980207443f,     0.02933359146118164f,     -0.09072602540254593f,     -0.021570585668087006f,     -0.011909959837794304f,     0.04913512244820595f,     -0.09793068468570709f,     -0.002443142468109727f,     -0.08733945339918137f,     -0.14329905807971954f,     -0.07344337552785873f,     -0.06117992848157883f,     -0.12335120141506195f,     -0.11269789189100266f,     -0.072536401450634f, 
    -0.22373560070991516f,     0.11588446795940399f,     -0.5950639247894287f,     0.18025563657283783f,     0.015537079423666f,     0.16119422018527985f,     0.2167471945285797f,     -0.3138871192932129f,     0.14867106080055237f,     0.2070891410112381f,     -0.12510691583156586f,     0.06058744341135025f,     -0.3208070993423462f,     0.23286543786525726f,     0.09612089395523071f,     -0.26029592752456665f, 
    0.0013207461452111602f,     -0.11762723326683044f,     0.10927775502204895f,     -0.07044132053852081f,     0.0074091944843530655f,     -0.1322803944349289f,     -0.1122695580124855f,     -0.015503089874982834f,     0.1284225434064865f,     -0.01348330918699503f,     -0.022446531802415848f,     -0.05346187949180603f,     -0.00833078846335411f,     -0.13955524563789368f,     -0.10716187953948975f,     -0.0161239355802536f, 
    0.06332708895206451f,     0.0420277938246727f,     -0.03135714679956436f,     -0.10678771883249283f,     -0.09700476378202438f,     -0.03517110273241997f,     -0.10234330594539642f,     -0.12225513160228729f,     0.08488104492425919f,     0.08200307190418243f,     -0.09802810847759247f,     0.07354201376438141f,     -0.024530641734600067f,     -0.1069645881652832f,     0.008953219279646873f,     -0.07430466264486313f, 
    0.2528400123119354f,     -0.010997996665537357f,     0.11183638125658035f,     0.14905160665512085f,     -0.17373748123645782f,     -0.02415706403553486f,     0.11638130992650986f,     0.1598900556564331f,     0.15928955376148224f,     -0.2166414111852646f,     0.10349303483963013f,     -0.17845629155635834f,     0.3033473491668701f,     -0.3235297203063965f,     -0.3457302153110504f,     -0.09928832948207855f, 
    0.17595237493515015f,     0.1548117846250534f,     -0.17883697152137756f,     0.2327006459236145f,     -0.12268742173910141f,     0.027149779722094536f,     0.1828804910182953f,     0.11633655428886414f,     0.13956575095653534f,     -0.0018472503870725632f,     0.2209162712097168f,     -0.3084401488304138f,     0.21755607426166534f,     -0.32030558586120605f,     -0.4099451005458832f,     -0.7331867814064026f, 
    -0.23594771325588226f,     0.14728368818759918f,     -0.974007785320282f,     -0.01929394342005253f,     0.24906587600708008f,     0.014354166574776173f,     0.188380166888237f,     -0.238748237490654f,     0.23249004781246185f,     0.31191033124923706f,     -0.27236446738243103f,     0.23569394648075104f,     -0.2757307291030884f,     0.12017261981964111f,     0.1536155492067337f,     -0.018019547685980797f, 
    0.0004934039898216724f,     0.05506656691431999f,     -0.026288019493222237f,     -0.0025004504714161158f,     -0.09682045876979828f,     0.03997286409139633f,     -0.04930492490530014f,     -0.00021044968161731958f,     -0.14890865981578827f,     -0.10851345211267471f,     -0.1530115306377411f,     0.01745862513780594f,     0.09978886693716049f,     0.06795338541269302f,     0.06001485511660576f,     -0.11238911002874374f, 
    -0.17894387245178223f,     0.15645699203014374f,     -0.2538844347000122f,     0.14456523954868317f,     0.09549007564783096f,     0.24522343277931213f,     0.23407861590385437f,     -0.1859060525894165f,     0.25384777784347534f,     0.14526450634002686f,     -0.1979541778564453f,     -0.032942138612270355f,     -0.21993011236190796f,     0.07685798406600952f,     -0.06473661959171295f,     0.4488242268562317f, 
    -0.18727564811706543f,     0.18974050879478455f,     -1.041143536567688f,     0.06709565222263336f,     0.33144620060920715f,     0.11253010481595993f,     0.1711128056049347f,     -0.2674216032028198f,     0.10884446650743484f,     0.2469661682844162f,     -0.2587602734565735f,     0.1817215085029602f,     -0.37891367077827454f,     0.3106071650981903f,     0.3136643171310425f,     -0.0962688997387886f, 
    0.13043956458568573f,     0.07615591585636139f,     -0.15191039443016052f,     0.04109546169638634f,     -0.05305585637688637f,     0.07251572608947754f,     -0.14815033972263336f,     -0.0454997755587101f,     -0.06577164679765701f,     -0.09685220569372177f,     -0.051182713359594345f,     -0.03626420348882675f,     -0.13676191866397858f,     0.06311936676502228f,     0.0967579111456871f,     0.009508893825113773f, 
    0.049650877714157104f,     0.13714618980884552f,     -0.3704203963279724f,     0.235613152384758f,     -0.20036451518535614f,     0.08085720241069794f,     0.2646598815917969f,     0.08566797524690628f,     0.1059575006365776f,     0.07379364967346191f,     0.1909227967262268f,     -0.4076958894729614f,     0.1069999486207962f,     -0.35748204588890076f,     -0.2314377874135971f,     -0.5531340837478638f, 
    0.21084058284759521f,     0.2584303915500641f,     0.10773815959692001f,     0.25642186403274536f,     -0.1741957664489746f,     0.11356429755687714f,     0.1094154641032219f,     0.15793642401695251f,     0.1530550718307495f,     0.10951659083366394f,     0.29204291105270386f,     -0.360431045293808f,     0.2825905680656433f,     -0.35410773754119873f,     -0.43199819326400757f,     -0.8930231332778931f, 
    -0.0587589368224144f,     -0.070826455950737f,     -0.04514699801802635f,     0.11223042011260986f,     -0.1734541654586792f,     0.028052236884832382f,     0.10822048783302307f,     0.07419116050004959f,     -0.0674198791384697f,     -0.21754729747772217f,     0.12751102447509766f,     -0.11597301065921783f,     -0.00686801690608263f,     -0.0054520717822015285f,     -0.26349371671676636f,     0.059768397361040115f, 
    -0.1243847981095314f,     -0.019822638481855392f,     -0.157609224319458f,     -0.08306774497032166f,     -0.028136640787124634f,     -0.11873705685138702f,     0.07267534732818604f,     -0.01628977060317993f,     0.015803635120391846f,     0.017400074750185013f,     0.07181381434202194f,     -0.02298402041196823f,     -0.007989716716110706f,     -0.12283705919981003f,     -0.08366422355175018f,     0.08893720805644989f, 
    0.22354623675346375f,     0.006287095136940479f,     -0.11378306150436401f,     0.2217945158481598f,     -0.1530819535255432f,     0.0771111473441124f,     0.12552286684513092f,     0.03062889724969864f,     0.19569720327854156f,     0.17077888548374176f,     0.2668546736240387f,     -0.3601953983306885f,     0.21563813090324402f,     -0.3526962995529175f,     -0.36152955889701843f,     -0.7475032210350037f, 
};

static const std::vector<float> LOUT_B = {
    0.2895067632198334f,     0.17685125768184662f,     0.056481145322322845f,     0.07503332942724228f,     0.17804820835590363f,     -0.09279891103506088f,     0.17566461861133575f,     0.21632005274295807f,     0.04181118309497833f,     -0.02756345644593239f,     0.14628039300441742f,     -0.1814713180065155f,     0.02318134717643261f,     -0.11466847360134125f,     -0.047761522233486176f,     0.08208071440458298f, 
    -0.027801046147942543f,     -0.053588442504405975f,     -0.08100266754627228f,     0.328340619802475f,     -0.03988619148731232f,     0.15698467195034027f,     0.13699474930763245f,     0.04322898015379906f,     0.035108547657728195f,     0.1506541222333908f,     0.14221566915512085f,     0.2916420102119446f,     0.12598322331905365f,     -0.014317410998046398f,     -0.05129947513341904f,     0.007240768987685442f, 
    -0.009769348427653313f,     0.3093470335006714f,     0.01297091692686081f,     -0.004204743541777134f,     -0.1377103626728058f,     0.2216363400220871f,     0.20206664502620697f,     0.009449184872210026f,     0.2780943214893341f,     0.2170467972755432f,     0.007095579523593187f,     0.24604962766170502f,     0.16817857325077057f,     0.07974345982074738f,     -0.017634350806474686f,     0.20141002535820007f, 
};

static const std::vector<float> MEAN_W = {
    -0.0304650217294693f,     0.1064520850777626f,     0.22327913343906403f,     -0.16084808111190796f,     -0.0532391332089901f,     -0.28910914063453674f,     -0.1614874303340912f,     -0.27503275871276855f,     0.23231665790081024f,     0.1664852797985077f,     0.09800013899803162f,     0.35449329018592834f,     -0.2012791484594345f,     0.15738900005817413f,     0.010540812276303768f,     0.13643702864646912f, 
    -0.10773380845785141f,     -0.1713205873966217f,     0.08512210100889206f,     0.03667585551738739f,     0.1339060217142105f,     -0.2074703872203827f,     -0.14119499921798706f,     0.18139557540416718f,     -0.0026773957069963217f,     0.10358941555023193f,     0.15967965126037598f,     0.000371112662833184f,     -0.1176847368478775f,     0.002696546260267496f,     -0.004641072824597359f,     0.06136985868215561f, 
    -0.0019232650520280004f,     0.23344072699546814f,     -0.0196500513702631f,     -0.0009827379835769534f,     -0.10112570971250534f,     -0.24616938829421997f,     0.3227965831756592f,     -0.00417298823595047f,     0.11217363178730011f,     0.34407392144203186f,     -0.0023465596605092287f,     -0.24297915399074554f,     -0.2898158133029938f,     -0.03086862526834011f,     -0.013300273567438126f,     -0.24540601670742035f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
