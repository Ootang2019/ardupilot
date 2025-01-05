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
    -0.6181923151016235f,
    -0.017766661942005157f,
    1.0f,
    -0.026383118703961372f,
    0.002580949803814292f,
    -0.794268786907196f,
    -0.4466763138771057f,
    0.07499891519546509f,
    0.4575766623020172f,
    0.42691686749458313f,
    -0.4926716685295105f,
    -0.5251381993293762f,
    0.45475614070892334f,
    -0.5437911152839661f,
    0.43308502435684204f,
    0.5489028692245483f,
    -0.3597264587879181f,
    -0.3652324378490448f,
    -0.44123905897140503f,
    1.0f,
    0.012356549501419067f,
    0.05494186654686928f,
    1.0f,
    0.017391378059983253f,
    -0.5038697123527527f,
    -0.6449298858642578f,
    0.017117835581302643f,
    0.4755721688270569f,
    0.4491588771343231f,
    -0.4619326591491699f,
    -0.4890764653682709f,
    0.447243332862854f,
    -0.5417437553405762f,
    0.42542779445648193f,
    -0.34064313769340515f,
    0.5621083378791809f,
    -0.33195167779922485f,
    -0.30837294459342957f,
    -0.23216457664966583f,
    -0.05655848979949951f,
    -0.03462487831711769f,
    0.06267828494310379f,
    1.0f,
    -0.22276920080184937f,
    -0.2688416540622711f,
    -1.0f,
    -0.5419337749481201f,
    -0.5735253691673279f,
    0.4844144284725189f,
    0.4249599277973175f,
    0.4088280200958252f,
    -0.5717630982398987f,
    0.3942672610282898f,
    -0.3129507601261139f,
    -0.28998494148254395f,
    0.6235300302505493f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.551826000213623f, 
    0.31425362825393677f, 
    0.17782798409461975f, 
    -0.21211747825145721f, 
    0.15696649253368378f, 
    0.35805079340934753f, 
    -0.26717662811279297f, 
    -0.4850791394710541f, 
    -0.22786441445350647f, 
    -0.24040591716766357f, 
    -0.045302391052246094f, 
    0.4411516487598419f, 
    0.4749266803264618f, 
    0.3337193429470062f, 
    -0.32594355940818787f, 
    -0.2396080195903778f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.031736962497234344f,
    -0.019040143117308617f,
    -0.09651950746774673f,
    0.07565569877624512f,
    -0.06480766087770462f,
    0.023609936237335205f,
    -0.1250598281621933f,
    -0.024370761588215828f,
    -0.049766357988119125f,
    0.043074123561382294f,
    -0.10931752622127533f,
    0.21986494958400726f,
    -0.09978503733873367f,
    -0.07102841138839722f,
    -0.11695405840873718f,
    0.0050510382279753685f,
};

static const std::vector<float> ANG_EMB_W = {
    -0.5063350200653076f, 
    1.1683318614959717f, 
    -0.07180310040712357f, 
    0.9246035814285278f, 
    -1.1500803232192993f, 
    -0.8234118223190308f, 
    0.1455494612455368f, 
    0.9598605036735535f, 
    -1.380302906036377f, 
    -0.23056484758853912f, 
    0.4558509290218353f, 
    0.46117886900901794f, 
    -0.06517653167247772f, 
    0.5996444821357727f, 
    0.4223809242248535f, 
    -1.7970645427703857f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.03559340164065361f,
    -0.01755085028707981f,
    -0.08599067479372025f,
    0.07420806586742401f,
    -0.06162894144654274f,
    0.0276289414614439f,
    -0.10843854397535324f,
    -0.028917893767356873f,
    -0.04631248861551285f,
    0.041655637323856354f,
    -0.10332407057285309f,
    0.16574926674365997f,
    -0.0896393284201622f,
    -0.06755417585372925f,
    -0.10786433517932892f,
    0.00656278058886528f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    -2.426440715789795f, 
    1.8469740152359009f, 
    -0.6547239422798157f, 
    1.9424740076065063f, 
    -2.7523930072784424f, 
    -1.8191951513290405f, 
    0.8274010419845581f, 
    3.0387308597564697f, 
    -2.1623053550720215f, 
    -1.557031273841858f, 
    0.7533459067344666f, 
    0.015083816833794117f, 
    -1.750579833984375f, 
    1.6732357740402222f, 
    0.8002813458442688f, 
    -3.4068071842193604f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.03060114197432995f,
    -0.029098689556121826f,
    0.013443093746900558f,
    0.021815303713083267f,
    -0.03694542497396469f,
    0.02496246062219143f,
    0.015934858471155167f,
    -0.025082143023610115f,
    -0.027511734515428543f,
    0.021956929937005043f,
    0.016950158402323723f,
    -0.12472032755613327f,
    -0.02256922237575054f,
    0.010726225562393665f,
    0.007962387055158615f,
    0.009434705600142479f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.35983556509017944f, 
    -0.1428951919078827f, 
    0.33688879013061523f, 
    -0.09767831861972809f, 
    0.23583446443080902f, 
    -0.0016117559280246496f, 
    -0.009034830145537853f, 
    -0.53531414270401f, 
    -0.3831973969936371f, 
    0.04241931065917015f, 
    0.04200295731425285f, 
    -0.5274761915206909f, 
    -0.0474848747253418f, 
    0.3629833459854126f, 
    -0.03587927669286728f, 
    -0.24628296494483948f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.02699539251625538f,
    0.013106433674693108f,
    0.08505751937627792f,
    -0.06844869256019592f,
    0.06481753289699554f,
    -0.021714916452765465f,
    0.1193416640162468f,
    0.02407658100128174f,
    0.046904996037483215f,
    -0.04530846327543259f,
    0.10189229249954224f,
    -0.2210630476474762f,
    0.09645083546638489f,
    0.06350770592689514f,
    0.1019814983010292f,
    -0.0058207144029438496f,
};

static const std::vector<float> GCN0_W = {
    -0.13266333937644958f,     0.32096317410469055f,     -0.08510365337133408f,     -0.16783690452575684f,     -0.5233270525932312f,     -0.2970312833786011f,     0.47864216566085815f,     0.4165079593658447f,     -0.5741780996322632f,     -0.3258599638938904f,     0.09312661737203598f,     -0.03063824214041233f,     0.01000894233584404f,     0.4968020021915436f,     0.3899953365325928f,     -0.28035879135131836f, 
    -0.35893338918685913f,     0.08488732576370239f,     0.377264142036438f,     -0.48840612173080444f,     -0.0737052708864212f,     -0.15336912870407104f,     -0.23987242579460144f,     -0.013342037796974182f,     -0.003942777402698994f,     -0.1156463474035263f,     0.06107316538691521f,     -0.4636213183403015f,     -0.06838159263134003f,     -0.0992516353726387f,     0.36071455478668213f,     0.38472625613212585f, 
    0.3340761959552765f,     -0.44274523854255676f,     -0.003339379793033004f,     -0.18602819740772247f,     -0.178555428981781f,     0.21372565627098083f,     -0.2150082141160965f,     -0.16896215081214905f,     0.535024106502533f,     -0.056191205978393555f,     -1.395480751991272f,     0.27395787835121155f,     -0.497071236371994f,     -0.8438134789466858f,     -1.059677243232727f,     0.3079371154308319f, 
    -0.5441595911979675f,     -0.23479869961738586f,     0.21041370928287506f,     -0.11836955696344376f,     0.04831711947917938f,     -0.2721855044364929f,     0.3916608989238739f,     0.4479754865169525f,     0.2541312873363495f,     0.012471817433834076f,     -0.09215087443590164f,     -0.3598940074443817f,     0.15567685663700104f,     0.13735997676849365f,     -0.3098941743373871f,     -0.4735826849937439f, 
    0.2538183331489563f,     -0.0036178003065288067f,     0.4637804925441742f,     -0.2956198751926422f,     0.5414074659347534f,     0.44021788239479065f,     0.4048862159252167f,     -0.776674747467041f,     0.527242124080658f,     -0.1363002210855484f,     0.3620114326477051f,     0.01751749962568283f,     0.0771193876862526f,     -0.26787686347961426f,     0.040452245622873306f,     1.2299638986587524f, 
    0.2095208764076233f,     -0.14917995035648346f,     0.2432594895362854f,     0.15294180810451508f,     0.32412445545196533f,     0.4006095826625824f,     0.08038905262947083f,     -0.10767320543527603f,     -0.06108729913830757f,     -0.45509088039398193f,     0.46242615580558777f,     -0.2831704318523407f,     -0.022840972989797592f,     -0.09435683488845825f,     0.1861131191253662f,     -0.32157739996910095f, 
    0.4071742594242096f,     -0.06853316724300385f,     0.5154515504837036f,     -0.16498412191867828f,     0.20096564292907715f,     -0.24241630733013153f,     0.46238216757774353f,     -0.3091438114643097f,     0.4042433500289917f,     0.023539556190371513f,     0.5250551700592041f,     -0.17413179576396942f,     0.6354638338088989f,     0.40320947766304016f,     0.3921477496623993f,     -0.14620649814605713f, 
    -0.17954133450984955f,     0.3020005524158478f,     -0.26693591475486755f,     0.6117608547210693f,     -0.2869899570941925f,     0.04481588304042816f,     0.19294266402721405f,     0.2952248752117157f,     -0.47777462005615234f,     -0.26187944412231445f,     0.5660631060600281f,     -0.31299030780792236f,     0.02662927284836769f,     0.5341728925704956f,     0.5916262269020081f,     -1.1750125885009766f, 
    -0.09878500550985336f,     0.5181316137313843f,     0.38488516211509705f,     0.294681191444397f,     -0.5083150267601013f,     -0.49929139018058777f,     -0.135676309466362f,     0.7488804459571838f,     -0.27429482340812683f,     -0.276009738445282f,     0.14427435398101807f,     -0.3924955129623413f,     0.2784856855869293f,     0.07250592857599258f,     -0.33396273851394653f,     -0.8263190984725952f, 
    0.47017908096313477f,     0.02418038249015808f,     -0.10593733936548233f,     -0.3854852616786957f,     0.46336135268211365f,     0.4130493700504303f,     0.2806030511856079f,     -0.428485244512558f,     0.37036722898483276f,     0.3276956081390381f,     0.13102729618549347f,     -0.1694132387638092f,     0.15240275859832764f,     0.13632558286190033f,     -0.25709789991378784f,     0.6917582154273987f, 
    -0.244590625166893f,     -0.2551017701625824f,     0.11301272362470627f,     -0.2934756875038147f,     0.08398357778787613f,     -0.46232202649116516f,     0.002438734984025359f,     0.6417951583862305f,     0.1612587869167328f,     -0.2755902409553528f,     0.06137269362807274f,     0.2970244586467743f,     0.19633875787258148f,     -0.19137834012508392f,     0.05967799574136734f,     -0.507735550403595f, 
    -0.0667039155960083f,     -0.5832350850105286f,     -0.05153892934322357f,     -0.404329389333725f,     0.3905033767223358f,     -0.04381865635514259f,     0.0916551798582077f,     -0.5132046937942505f,     0.5190437436103821f,     -0.08804264664649963f,     -0.27623167634010315f,     -0.30394524335861206f,     0.5092169642448425f,     0.09458509087562561f,     0.0864056944847107f,     1.3329817056655884f, 
    -0.028264516964554787f,     0.5754290223121643f,     0.17416246235370636f,     0.09292707592248917f,     -0.5400499105453491f,     0.09158284962177277f,     0.5037405490875244f,     0.37797635793685913f,     -0.6042684316635132f,     -0.16274648904800415f,     0.41288748383522034f,     -0.1267935335636139f,     -0.17010176181793213f,     0.008892198093235493f,     0.016422048211097717f,     -1.0871515274047852f, 
    0.06685440242290497f,     0.2877415418624878f,     0.47261735796928406f,     0.08930230140686035f,     0.19983239471912384f,     0.40051770210266113f,     0.4326635003089905f,     -0.06604383885860443f,     0.2596246302127838f,     0.17804697155952454f,     -0.10434234887361526f,     -0.14501987397670746f,     0.2996698319911957f,     0.2149941474199295f,     0.013489405624568462f,     0.5076847672462463f, 
    0.5809827446937561f,     -0.18911506235599518f,     0.22388149797916412f,     0.00663910573348403f,     0.12143590301275253f,     0.16109810769557953f,     -0.09351860731840134f,     -0.895015299320221f,     0.45407840609550476f,     -0.3420834541320801f,     0.23466800153255463f,     -0.2843010127544403f,     0.19313088059425354f,     -0.30803149938583374f,     -0.0005361193907447159f,     0.599897027015686f, 
    0.12927675247192383f,     0.04665728658437729f,     -0.3897168040275574f,     0.6984427571296692f,     -0.43273910880088806f,     0.020802753046154976f,     0.10780899971723557f,     -0.4135702848434448f,     -0.47876062989234924f,     0.2688208818435669f,     -0.10272880643606186f,     0.49514660239219666f,     0.18252308666706085f,     -0.2988029718399048f,     0.026026248931884766f,     -0.31535595655441284f, 
};

static const std::vector<float> GCN0_B = {
    0.0401364304125309f,
    0.12439405918121338f,
    -0.30744773149490356f,
    0.08352800458669662f,
    0.06072705611586571f,
    0.08167700469493866f,
    0.18533651530742645f,
    0.04237570986151695f,
    0.04026331380009651f,
    0.053332868963479996f,
    0.08395108580589294f,
    0.05548657849431038f,
    0.08852966129779816f,
    0.12576553225517273f,
    0.03739270195364952f,
    -0.1847842037677765f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.04434557631611824f,     -0.03591988608241081f,     -0.02591031789779663f,     0.01641634851694107f,     0.18195536732673645f,     -0.034318771213293076f,     0.09385062009096146f,     0.06060121953487396f,     0.0645013302564621f,     0.18634484708309174f,     0.03828581050038338f,     0.04649372771382332f,     0.07544051110744476f,     -0.0007405465003103018f,     0.013054635375738144f,     -0.01008493173867464f, 
    -0.05073193088173866f,     0.19518141448497772f,     -0.22200565040111542f,     0.19304904341697693f,     0.23128776252269745f,     0.171480193734169f,     0.08275985717773438f,     -0.3167988657951355f,     -0.26246213912963867f,     0.16107402741909027f,     0.12857036292552948f,     0.034956224262714386f,     -0.3106277287006378f,     0.1250932365655899f,     0.2088286429643631f,     0.08946805447340012f, 
    -0.09155318140983582f,     0.147362619638443f,     -0.158144012093544f,     0.17652614414691925f,     0.22263240814208984f,     0.07140182703733444f,     -0.0024206216912716627f,     -0.13321125507354736f,     -0.14381244778633118f,     0.08396338671445847f,     -0.07051651179790497f,     0.1840868443250656f,     -0.09943914413452148f,     0.16594883799552917f,     -0.02767840400338173f,     -0.05436259135603905f, 
    0.06004135683178902f,     0.15043982863426208f,     -0.04365302622318268f,     0.03151370584964752f,     -0.054590027779340744f,     0.10023705661296844f,     0.1399267017841339f,     0.06164775788784027f,     0.1713441163301468f,     -0.1686326414346695f,     0.08429711312055588f,     -0.32330796122550964f,     0.21257559955120087f,     0.17380301654338837f,     -0.28776854276657104f,     -0.20833738148212433f, 
    0.08602060377597809f,     -0.007944758050143719f,     -0.021373538300395012f,     0.07516700774431229f,     -0.2724878191947937f,     -0.18032096326351166f,     -0.034785039722919464f,     0.15634150803089142f,     0.1761663407087326f,     -0.185649111866951f,     0.09017498046159744f,     -0.22311611473560333f,     -0.07196745276451111f,     0.026014138013124466f,     -0.12241160124540329f,     -0.04934350773692131f, 
    -0.03753108158707619f,     0.08363700658082962f,     -0.2005772441625595f,     0.03269784525036812f,     0.16642996668815613f,     0.07471086084842682f,     0.13396747410297394f,     -0.264445960521698f,     -0.26668888330459595f,     0.043158553540706635f,     0.21205870807170868f,     0.18009206652641296f,     -0.12384446710348129f,     0.1551501601934433f,     0.11535200476646423f,     -0.07433682680130005f, 
    0.17894378304481506f,     0.12009979039430618f,     0.040602535009384155f,     0.07661212980747223f,     -0.1754373461008072f,     0.06372427940368652f,     0.1497347354888916f,     0.02566409669816494f,     0.07978561520576477f,     -0.23316679894924164f,     0.060659393668174744f,     -0.2972552180290222f,     0.23222114145755768f,     0.1236669048666954f,     -0.05166802927851677f,     -0.0538419745862484f, 
    0.031216569244861603f,     0.20609864592552185f,     -0.0992368683218956f,     0.0657225251197815f,     -0.08293604105710983f,     -0.0007840408943593502f,     0.08291980624198914f,     0.2560737133026123f,     0.08469769358634949f,     -0.07474281638860703f,     0.1609191596508026f,     -0.28093385696411133f,     0.18379290401935577f,     -0.009962380863726139f,     -0.22257764637470245f,     -0.12906014919281006f, 
    -0.12155259400606155f,     0.17432911694049835f,     -0.3382401466369629f,     -0.025986967608332634f,     0.16919448971748352f,     0.11493311822414398f,     0.14240068197250366f,     -0.288404256105423f,     -0.009686554782092571f,     0.1951574683189392f,     0.06382962316274643f,     0.19419066607952118f,     -0.10929358005523682f,     0.08163982629776001f,     0.1827908158302307f,     0.043838366866111755f, 
    -0.6038620471954346f,     -0.029947904869914055f,     0.27342647314071655f,     -0.1883319914340973f,     0.010687864385545254f,     -0.07514628022909164f,     -0.2739471197128296f,     -0.0671263337135315f,     -0.020309116691350937f,     0.03545765578746796f,     -0.1619880348443985f,     0.14385998249053955f,     0.11800862848758698f,     -0.09162677079439163f,     0.050300292670726776f,     -0.09494809061288834f, 
    -0.04210503771901131f,     0.056788504123687744f,     0.03386690840125084f,     0.09995406866073608f,     -0.15061073005199432f,     0.09527788311243057f,     0.042314570397138596f,     -0.01726638525724411f,     0.16381080448627472f,     -0.04422628879547119f,     0.11347400397062302f,     -0.28323593735694885f,     0.21595551073551178f,     0.16610227525234222f,     -0.027825690805912018f,     -0.07321708649396896f, 
    -0.000695869792252779f,     -0.25237372517585754f,     -0.06272150576114655f,     -0.2643830478191376f,     -0.3086574077606201f,     -0.025973854586482048f,     -0.2051704078912735f,     0.06793992221355438f,     0.053873930126428604f,     -0.12729060649871826f,     -0.07375616580247879f,     -0.008471430279314518f,     0.06981877982616425f,     -0.376395046710968f,     -0.11707253009080887f,     0.22246302664279938f, 
    0.19476249814033508f,     -0.08498405665159225f,     0.06808485090732574f,     0.12321493774652481f,     -0.19577144086360931f,     0.19236691296100616f,     0.17870411276817322f,     0.2028583586215973f,     0.152553528547287f,     -0.10846566408872604f,     0.07837823033332825f,     -0.12443746626377106f,     0.22160939872264862f,     -0.010775839909911156f,     -0.09317505359649658f,     -0.027501782402396202f, 
    -0.026156721636652946f,     -0.17660152912139893f,     0.03561573848128319f,     -0.15461455285549164f,     -0.3937736451625824f,     -0.025896834209561348f,     -0.19357827305793762f,     0.09245426207780838f,     0.04225914180278778f,     0.015283655375242233f,     -0.2552163004875183f,     0.09954830259084702f,     0.08959026634693146f,     -0.3204190135002136f,     -0.1538565754890442f,     0.12682250142097473f, 
    -0.0933246836066246f,     -0.06796085089445114f,     0.06887227296829224f,     0.10746710002422333f,     0.06584162265062332f,     0.07377772778272629f,     -0.049066074192523956f,     -0.1167205348610878f,     -0.10484900325536728f,     -0.12184428423643112f,     -0.03778335824608803f,     -0.15993328392505646f,     0.11179541796445847f,     -0.09181464463472366f,     -0.009571335278451443f,     -0.0700145810842514f, 
    -0.0016728362534195185f,     0.010284321382641792f,     -0.2416694462299347f,     -0.09662970900535583f,     0.14697642624378204f,     -0.07528069615364075f,     -0.004780316725373268f,     -0.45169955492019653f,     -0.21090558171272278f,     0.1406005173921585f,     -0.03566110134124756f,     0.21919527649879456f,     -0.2830214202404022f,     0.09928061068058014f,     0.17310790717601776f,     0.08087420463562012f, 
    0.20618686079978943f,     -0.06103520840406418f,     -0.011280221864581108f,     0.12578926980495453f,     -0.056214749813079834f,     0.1860157698392868f,     0.008927937597036362f,     0.0702846348285675f,     0.1690858006477356f,     -0.06756149977445602f,     0.17183136940002441f,     -0.27268752455711365f,     0.1807498186826706f,     0.0835590660572052f,     -0.13127648830413818f,     -0.5052912831306458f, 
    0.19924698770046234f,     0.04088263586163521f,     0.539569616317749f,     0.13634148240089417f,     -0.18969987332820892f,     0.10162266343832016f,     0.09994940459728241f,     0.18912909924983978f,     0.21144476532936096f,     -0.267863929271698f,     0.03585870563983917f,     -0.10082866251468658f,     0.14558365941047668f,     0.15547087788581848f,     -0.13342584669589996f,     -0.44689181447029114f, 
    -0.0616874136030674f,     0.0904582142829895f,     -0.284260094165802f,     0.004645467270165682f,     0.03955031558871269f,     -0.02530667372047901f,     0.15794602036476135f,     -0.07579569518566132f,     -0.25972214341163635f,     -0.04690847918391228f,     0.11651841551065445f,     0.18584668636322021f,     -0.21824999153614044f,     0.13325348496437073f,     0.14919854700565338f,     0.18895074725151062f, 
    -0.12138690799474716f,     -0.06463094800710678f,     -0.1393386423587799f,     0.03240596503019333f,     0.14461392164230347f,     0.07565361261367798f,     -0.03429686650633812f,     -0.32296010851860046f,     -0.15990427136421204f,     0.18851718306541443f,     0.036431316286325455f,     0.09379998594522476f,     -0.2947366535663605f,     -0.04743549972772598f,     -0.006536693777889013f,     0.07068870961666107f, 
    -0.16979706287384033f,     -0.005680026486515999f,     -0.01267113909125328f,     0.08159610629081726f,     -0.13204419612884521f,     -0.06180253252387047f,     0.0419132374227047f,     -0.016730282455682755f,     -0.030445098876953125f,     -0.1277529001235962f,     -0.08572176843881607f,     0.1070299819111824f,     -0.021028252318501472f,     0.020967721939086914f,     0.056400373578071594f,     -0.0917220264673233f, 
    0.19321241974830627f,     0.005010745022445917f,     0.007305526174604893f,     0.2371068000793457f,     -0.18458512425422668f,     0.21497736871242523f,     0.06053641438484192f,     0.14424684643745422f,     0.10013343393802643f,     -0.12395542860031128f,     0.18215356767177582f,     -0.2712780833244324f,     0.0400647297501564f,     0.009499757550656796f,     -0.2760527431964874f,     -0.6658085584640503f, 
    0.11364955455064774f,     0.03759385645389557f,     -0.0883924663066864f,     0.0318911038339138f,     -0.12238600850105286f,     -0.020719027146697044f,     -0.008787397295236588f,     0.11496511846780777f,     0.14120641350746155f,     -0.32726356387138367f,     0.12854912877082825f,     -0.347206711769104f,     0.17966629564762115f,     -0.046568628400564194f,     -0.19232285022735596f,     -0.5097630620002747f, 
    -0.09148021787405014f,     0.05303715541958809f,     -0.36045607924461365f,     0.11370427161455154f,     0.2570807635784149f,     0.20376558601856232f,     0.1856810450553894f,     -0.330221563577652f,     -0.1220671534538269f,     0.21519680321216583f,     -0.012176550924777985f,     0.2635653614997864f,     -0.22798855602741241f,     0.20822997391223907f,     0.11788693815469742f,     0.03512122109532356f, 
    0.09074368327856064f,     -0.04358622059226036f,     -0.023169802501797676f,     -0.05300876498222351f,     -0.01655537448823452f,     -0.022794997319579124f,     0.023312265053391457f,     -0.09066235274076462f,     -0.049081433564424515f,     0.04708421975374222f,     -0.016481276601552963f,     -0.13364142179489136f,     0.10391020029783249f,     0.06829424947500229f,     0.03127032890915871f,     -0.11678304523229599f, 
    -0.12838515639305115f,     0.16214582324028015f,     -0.24962115287780762f,     0.01572306640446186f,     0.14086562395095825f,     0.126273050904274f,     0.13725367188453674f,     -0.2985823452472687f,     -0.14702188968658447f,     0.0585654191672802f,     0.06324997544288635f,     0.0455092191696167f,     -0.23808954656124115f,     0.15838086605072021f,     0.04723457619547844f,     -0.08712761849164963f, 
    -0.09436862915754318f,     0.0695662647485733f,     -0.380778044462204f,     0.008258050307631493f,     0.21359479427337646f,     0.1801082044839859f,     0.12240774929523468f,     -0.17056332528591156f,     -0.09463248401880264f,     0.1685774028301239f,     -0.0203301552683115f,     0.2878730595111847f,     -0.27603092789649963f,     0.12662965059280396f,     0.2362498641014099f,     -0.022506054490804672f, 
    0.13236337900161743f,     0.15373730659484863f,     0.44101855158805847f,     0.049439407885074615f,     -0.02368878945708275f,     0.042630694806575775f,     0.11751948297023773f,     0.07287531346082687f,     0.07059146463871002f,     -0.09189373254776001f,     -0.04288344457745552f,     -0.14191490411758423f,     0.011394522152841091f,     0.05648268014192581f,     -0.20443294942378998f,     -0.12856800854206085f, 
    0.12609243392944336f,     0.17564958333969116f,     0.11258795112371445f,     0.029162747785449028f,     -0.1132756918668747f,     0.11350174993276596f,     0.1517164558172226f,     0.21324187517166138f,     0.08553101122379303f,     -0.30291131138801575f,     0.07135090976953506f,     -0.403716504573822f,     0.19843240082263947f,     0.1710129976272583f,     -0.11973711103200912f,     -0.5703871846199036f, 
    0.1013428345322609f,     -0.13333910703659058f,     0.019336391240358353f,     -0.1666577309370041f,     0.10155203938484192f,     -0.15448196232318878f,     -0.0713929831981659f,     -0.08369861543178558f,     -0.012550272047519684f,     -0.023643918335437775f,     0.013865595683455467f,     -0.08923455327749252f,     -0.03510419279336929f,     -0.03046536073088646f,     -0.050121936947107315f,     -0.1219775453209877f, 
    -0.1083478033542633f,     0.08173423260450363f,     0.10779349505901337f,     -0.07444378733634949f,     0.11551041156053543f,     0.00462378142401576f,     -0.13827729225158691f,     0.11612191051244736f,     -0.146418035030365f,     -0.11388864368200302f,     -0.06888847798109055f,     -0.13653075695037842f,     0.02303159609436989f,     -0.05736790597438812f,     -0.0264393649995327f,     0.05611788481473923f, 
    -0.015696104615926743f,     -0.0535222664475441f,     -0.2422976940870285f,     -0.03175907954573631f,     0.0862831398844719f,     -0.06552162766456604f,     0.1267063468694687f,     -0.06775950640439987f,     -0.0681905522942543f,     0.08862417191267014f,     0.08978477120399475f,     0.21711772680282593f,     -0.10272938758134842f,     0.005201093852519989f,     -0.013554471544921398f,     0.4895791709423065f, 
    0.05753910914063454f,     0.04202307388186455f,     0.02933359146118164f,     -0.10128341615200043f,     -0.059588152915239334f,     -0.03178057074546814f,     0.033378761261701584f,     -0.10650802403688431f,     0.028146108612418175f,     -0.13086998462677002f,     -0.1112346202135086f,     -0.08797876536846161f,     -0.06618775427341461f,     -0.13551218807697296f,     -0.12356223911046982f,     -0.10484983772039413f, 
    0.11067096889019012f,     0.01116221584379673f,     -0.010234829969704151f,     0.10699053853750229f,     -0.24872663617134094f,     0.0384611077606678f,     0.0830654427409172f,     0.042398083955049515f,     0.012097327038645744f,     -0.20579636096954346f,     0.21741658449172974f,     -0.31502243876457214f,     0.05274640768766403f,     0.165658101439476f,     -0.24827584624290466f,     -0.3288080096244812f, 
    -0.013733276166021824f,     -0.13454556465148926f,     0.11110761761665344f,     -0.12480250000953674f,     0.010363772511482239f,     -0.13271909952163696f,     -0.14211249351501465f,     -0.028670212253928185f,     0.11817866563796997f,     -0.008818651549518108f,     -0.028847048059105873f,     -0.06684641540050507f,     -0.021206116303801537f,     -0.13708193600177765f,     -0.10353156924247742f,     0.017166707664728165f, 
    0.06345925480127335f,     0.042160872370004654f,     -0.03135368227958679f,     -0.10698186606168747f,     -0.09691682457923889f,     -0.03517110273241997f,     -0.10220223665237427f,     -0.1222156435251236f,     0.08492019027471542f,     0.08211095631122589f,     -0.0980043038725853f,     0.07364963740110397f,     -0.02451949380338192f,     -0.10684653371572495f,     0.009040075354278088f,     -0.0742545947432518f, 
    0.21356868743896484f,     0.007799185812473297f,     0.08125724643468857f,     0.21213968098163605f,     -0.13091608881950378f,     0.08027684688568115f,     0.21439097821712494f,     0.2023439109325409f,     0.26179301738739014f,     -0.20139960944652557f,     0.04440615698695183f,     -0.3253065347671509f,     0.30364131927490234f,     -0.09604492783546448f,     -0.3687596917152405f,     -0.3174454867839813f, 
    0.09627485275268555f,     -0.02103421464562416f,     -0.10923785716295242f,     0.02710821107029915f,     0.013085423037409782f,     -0.18086709082126617f,     0.05279768258333206f,     0.010366187430918217f,     0.05577892065048218f,     -0.26445290446281433f,     0.0706140473484993f,     -0.22498300671577454f,     0.14305754005908966f,     -0.04844360426068306f,     -0.2911720871925354f,     -0.08762107789516449f, 
    -0.05486232414841652f,     0.11673825234174728f,     -0.6633200645446777f,     -0.0486316904425621f,     0.21886850893497467f,     0.04853568226099014f,     0.123319111764431f,     -0.23789620399475098f,     -0.14217153191566467f,     0.2376934289932251f,     0.01823294349014759f,     0.2563973665237427f,     -0.25728946924209595f,     0.03472823649644852f,     0.1467936635017395f,     0.0658140704035759f, 
    -0.04364363104104996f,     0.028284020721912384f,     0.058193013072013855f,     -0.033986106514930725f,     -0.07649203389883041f,     0.027521302923560143f,     -0.07315170019865036f,     -0.033838290721178055f,     -0.19647859036922455f,     -0.08846168965101242f,     -0.18178540468215942f,     0.030896436423063278f,     0.05168462544679642f,     0.07486017793416977f,     0.08259770274162292f,     -0.10239491611719131f, 
    0.028210269287228584f,     0.012317507527768612f,     -0.0226754043251276f,     -0.017245866358280182f,     0.049987226724624634f,     0.054560501128435135f,     0.11005060374736786f,     -0.010337910614907742f,     0.12654392421245575f,     0.04620031639933586f,     0.03521103039383888f,     -0.06469829380512238f,     0.004773156251758337f,     0.04466470703482628f,     -0.10726983845233917f,     0.07271149009466171f, 
    -0.002103438600897789f,     0.14330941438674927f,     -0.5678823590278625f,     0.011262660846114159f,     0.2753792405128479f,     0.15030847489833832f,     0.0911203920841217f,     -0.29735371470451355f,     -0.2902313470840454f,     0.15308155119419098f,     0.053542036563158035f,     0.16927051544189453f,     -0.3774520456790924f,     0.18564827740192413f,     0.2565607726573944f,     0.03233901783823967f, 
    0.25645971298217773f,     0.17813782393932343f,     -0.2641080617904663f,     0.17140324413776398f,     -0.20180116593837738f,     0.19373446702957153f,     -0.04884996637701988f,     0.1437852829694748f,     0.10267265141010284f,     -0.30461621284484863f,     0.07677324116230011f,     -0.34504520893096924f,     0.055368486791849136f,     0.14537981152534485f,     -0.1683691293001175f,     -0.23788459599018097f, 
    -0.11904023587703705f,     0.05553722381591797f,     -0.35008731484413147f,     0.0960901603102684f,     0.1622866988182068f,     -0.03146011009812355f,     0.19610904157161713f,     -0.28211709856987f,     -0.2613075375556946f,     0.1105988398194313f,     0.08539778739213943f,     0.01082860492169857f,     -0.24618642032146454f,     0.006436577066779137f,     0.23814113438129425f,     0.03625546395778656f, 
    0.08436552435159683f,     0.18096011877059937f,     0.06247870251536369f,     0.16924898326396942f,     -0.07395747303962708f,     0.007160671055316925f,     0.04996300861239433f,     0.027833469212055206f,     0.10133618116378784f,     -0.16556265950202942f,     0.14720606803894043f,     -0.3529391288757324f,     0.15789920091629028f,     -0.015077152289450169f,     -0.40324437618255615f,     -0.4420711398124695f, 
    -0.09539284557104111f,     -0.1314794421195984f,     0.16809457540512085f,     0.03401746228337288f,     -0.05271537974476814f,     -0.07049858570098877f,     0.0939166322350502f,     0.01737140864133835f,     -0.09507311880588531f,     -0.12705421447753906f,     0.10678312927484512f,     -0.013360380195081234f,     -0.03826263174414635f,     0.13168689608573914f,     -0.1785569041967392f,     0.020486872643232346f, 
    -0.05776461213827133f,     -0.2725241482257843f,     -0.08781232684850693f,     -0.2651780843734741f,     -0.2609012722969055f,     -0.16663941740989685f,     -0.029632730409502983f,     0.11518234759569168f,     0.11106020212173462f,     0.007548963651061058f,     0.0007083369418978691f,     -0.11903797090053558f,     0.11751537024974823f,     -0.3457529544830322f,     -0.1290503889322281f,     0.39554810523986816f, 
    0.1475493311882019f,     -0.14869073033332825f,     0.42113518714904785f,     0.059567276388406754f,     -0.002542876871302724f,     -0.12091907858848572f,     -0.020979544147849083f,     -0.03334818035364151f,     0.1420625001192093f,     0.009242034517228603f,     0.1276206523180008f,     -0.18650779128074646f,     0.1713089644908905f,     -0.08624783903360367f,     -0.1514633595943451f,     -0.20672400295734406f, 
};

static const std::vector<float> LOUT_B = {
    0.07216008752584457f,     0.09384025633335114f,     0.06486755609512329f,     0.05588734522461891f,     -0.018562158569693565f,     0.08840242773294449f,     0.08032454550266266f,     0.12501370906829834f,     0.06379880756139755f,     -0.1399511992931366f,     0.07818882167339325f,     -0.14232002198696136f,     0.029989296570420265f,     -0.15791089832782745f,     -0.01225481741130352f,     0.02389332838356495f, 
    0.018148036673665047f,     0.05322986841201782f,     0.05871737375855446f,     0.02444240264594555f,     -0.028673261404037476f,     0.13202503323554993f,     0.06324120610952377f,     0.062109511345624924f,     0.029432330280542374f,     0.07022760808467865f,     0.12124636769294739f,     0.04818570241332054f,     0.07244396209716797f,     -0.014848824590444565f,     -0.012521730735898018f,     0.07340864092111588f, 
    -0.0151147386059165f,     0.1156393364071846f,     0.010073825716972351f,     -0.0041757868602871895f,     0.07267480343580246f,     0.036042604595422745f,     0.12131722271442413f,     -0.01970953308045864f,     0.0779339075088501f,     0.1478559523820877f,     0.11013314127922058f,     0.09007487446069717f,     0.06621254235506058f,     0.03213004767894745f,     -0.18886210024356842f,     0.0069497125223279f, 
};

static const std::vector<float> MEAN_W = {
    0.014760422520339489f,     0.1077020987868309f,     0.15115220844745636f,     -0.14793451130390167f,     -0.0713159516453743f,     0.23252776265144348f,     -0.11318162828683853f,     -0.20039255917072296f,     0.17942747473716736f,     -0.2746833860874176f,     -0.09225785732269287f,     0.23844881355762482f,     -0.1534828543663025f,     0.271577924489975f,     -0.013443328440189362f,     0.11787260323762894f, 
    -0.12492882460355759f,     -0.19567415118217468f,     0.12074098736047745f,     0.13153356313705444f,     -0.0028934190049767494f,     -0.21119937300682068f,     -0.15945066511631012f,     0.22833660244941711f,     -0.0009374921210110188f,     0.12771858274936676f,     0.20977742969989777f,     -0.040072035044431686f,     -0.14356867969036102f,     0.00694305170327425f,     0.007375708781182766f,     0.09497396647930145f, 
    -0.010867306031286716f,     -0.2109581083059311f,     0.05698351189494133f,     -0.0006938384613022208f,     -0.22280755639076233f,     -0.07899998873472214f,     0.2675153613090515f,     -0.00029520984389819205f,     -0.0012004008749499917f,     0.28857356309890747f,     -0.22959142923355103f,     0.2209576815366745f,     -0.17250946164131165f,     -0.00786496140062809f,     0.2657984793186188f,     -0.13440175354480743f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
