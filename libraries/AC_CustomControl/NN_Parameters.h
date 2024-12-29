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
static constexpr int N_HIDDEN = 8;

static constexpr float AVEL_LIM = 15.700000;
static constexpr float POS_LIM = 999999.900000;
static constexpr float AUTHORITY = 0.250000;

std::vector<float> OBS = {
0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, };
std::vector<float> TASK = {
0.250000, 0.250000, 0.250000, 0.250000, 0.000000, 0.000000, 0.000000, };
static const std::vector<float> A = {
    1.0f,
    -0.4557696580886841f,
    0.06721501052379608f,
    1.0f,
    0.06634138524532318f,
    0.11608002334833145f,
    -1.0f,
    -0.7420040369033813f,
    -0.11193924397230148f,
    0.5816352367401123f,
    0.5666395425796509f,
    -0.3527292311191559f,
    -0.3484661877155304f,
    0.5572117567062378f,
    -0.40849918127059937f,
    0.5844289064407349f,
    0.4395603835582733f,
    -0.48568424582481384f,
    -0.5122668147087097f,
    -0.6776314377784729f,
    1.0f,
    -0.13417211174964905f,
    -0.20312120020389557f,
    1.0f,
    0.06458204239606857f,
    -0.42218872904777527f,
    -1.0f,
    0.2808966338634491f,
    0.3642723262310028f,
    0.37942278385162354f,
    -0.6400801539421082f,
    -0.607017993927002f,
    0.43434080481529236f,
    -0.5867921113967896f,
    0.4761063754558563f,
    -0.3446941673755646f,
    0.47563436627388f,
    -0.36729195713996887f,
    -0.37505707144737244f,
    -0.337691068649292f,
    -0.003934526350349188f,
    -0.02056458406150341f,
    0.03204740211367607f,
    1.0f,
    -0.3978547751903534f,
    -0.3943406939506531f,
    -0.6358973383903503f,
    -0.5335605144500732f,
    -0.5268278121948242f,
    0.45182085037231445f,
    0.4130201041698456f,
    0.3629269301891327f,
    -0.6070991158485413f,
    0.37496232986450195f,
    -0.28562381863594055f,
    -0.26979148387908936f,
    0.6919443607330322f,
};

static const std::vector<float> ACT_EMB_W = {
    -0.43884146213531494f, 
    -0.069455586373806f, 
    -0.6285459995269775f, 
    0.9046920537948608f, 
    0.12219513952732086f, 
    1.002793788909912f, 
    0.2366836816072464f, 
    0.20641936361789703f, 
};

static const std::vector<float> ACT_EMB_B = {
    0.1785961091518402f,
    -0.2396230250597f,
    0.01940205693244934f,
    0.08167687803506851f,
    0.1050821840763092f,
    -0.10944101214408875f,
    0.07944447547197342f,
    0.0693034678697586f,
};

static const std::vector<float> ANG_EMB_W = {
    1.6772315502166748f, 
    0.4789782464504242f, 
    -1.9570633172988892f, 
    -0.7713155746459961f, 
    -0.45574137568473816f, 
    -0.3778347671031952f, 
    2.2943954467773438f, 
    -3.1414921283721924f, 
};

static const std::vector<float> ANG_EMB_B = {
    0.16453863680362701f,
    -0.2678677439689636f,
    0.029636377468705177f,
    0.1032276600599289f,
    0.1190631315112114f,
    -0.12847881019115448f,
    0.0696737989783287f,
    0.08592026680707932f,
};

static const std::vector<float> ANGVEL_EMB_W = {
    3.9190292358398438f, 
    0.6277739405632019f, 
    -4.596321105957031f, 
    -3.0849483013153076f, 
    -3.158208131790161f, 
    -3.9592227935791016f, 
    5.553038597106934f, 
    -4.1568498611450195f, 
};

static const std::vector<float> ANGVEL_EMB_B = {
    0.0024540810845792294f,
    0.04696499928832054f,
    0.030557336285710335f,
    0.033319588750600815f,
    0.021708285436034203f,
    -0.04437585175037384f,
    0.03663749620318413f,
    0.03133624419569969f,
};

static const std::vector<float> TASK_EMB_W = {
    -0.4568209946155548f, 
    0.8189244866371155f, 
    -0.018867967650294304f, 
    -0.0556161068379879f, 
    0.19259516894817352f, 
    -0.009704312309622765f, 
    0.20738890767097473f, 
    0.2822819650173187f, 
};

static const std::vector<float> TASK_EMB_B = {
    -0.1474781632423401f,
    0.20326319336891174f,
    -0.044471707195043564f,
    -0.11316042393445969f,
    -0.12673403322696686f,
    0.10105101019144058f,
    -0.06713048368692398f,
    -0.0937475934624672f,
};

static const std::vector<float> GCN0_W = {
    -0.7426867485046387f,     0.30558720231056213f,     0.648263692855835f,     0.23384617269039154f,     -0.07982311397790909f,     -0.03012070059776306f,     -1.2319010496139526f,     0.8494875431060791f, 
    0.20412985980510712f,     0.03705666586756706f,     -1.1267563104629517f,     -0.6207770109176636f,     -0.6662722229957581f,     -0.1751031130552292f,     1.245236873626709f,     -0.864960789680481f, 
    -0.6519960165023804f,     1.182544469833374f,     -0.1674199253320694f,     -0.5665419101715088f,     -0.5387506484985352f,     0.7754490375518799f,     -0.46029549837112427f,     -0.8316928148269653f, 
    -0.07400919497013092f,     -0.0017244340851902962f,     -0.8877982497215271f,     -0.5063627362251282f,     -0.3884468674659729f,     -0.20698022842407227f,     1.2963624000549316f,     -0.8116111755371094f, 
    1.8798776865005493f,     -0.34466299414634705f,     -0.5237591862678528f,     0.6170645952224731f,     0.5957015156745911f,     0.3796473741531372f,     1.4294685125350952f,     -0.7668347954750061f, 
    0.15591900050640106f,     0.4373002052307129f,     -0.06241578236222267f,     -0.07518763840198517f,     -0.31532979011535645f,     -0.21031829714775085f,     1.2767539024353027f,     -0.0860360711812973f, 
    -0.16895996034145355f,     0.2102905511856079f,     0.09677466005086899f,     -0.30926477909088135f,     -0.39301222562789917f,     0.7468438744544983f,     -1.0572962760925293f,     0.17780980467796326f, 
    0.6400586366653442f,     0.34079495072364807f,     -0.8808268904685974f,     -0.32454046607017517f,     -0.5182468891143799f,     -0.024590739980340004f,     0.3682175576686859f,     0.3069811165332794f, 
};

static const std::vector<float> GCN0_B = {
    0.11580514907836914f,
    0.10859821736812592f,
    0.5035853385925293f,
    0.027634229511022568f,
    -0.5611426830291748f,
    0.07470021396875381f,
    0.20413586497306824f,
    -0.10878174006938934f,
};

static const std::vector<float> GCN0_LN_W = {
};

static const std::vector<float> GCN0_LN_B = {
};

static const std::vector<float> LOUT_W = {
    -0.44590887427330017f,     0.1470668464899063f,     0.01478134747594595f,     0.19788016378879547f,     0.1761549413204193f,     0.07729335129261017f,     0.0094229094684124f,     0.20205390453338623f, 
    0.10922865569591522f,     -0.6260374784469604f,     0.2799028754234314f,     -0.2665749490261078f,     -0.26416105031967163f,     -0.34509339928627014f,     0.45288169384002686f,     0.04614483192563057f, 
    0.23565822839736938f,     -0.1406061351299286f,     0.3583817481994629f,     -0.33593660593032837f,     -0.23620407283306122f,     0.2984517216682434f,     0.10650552064180374f,     -0.11036550998687744f, 
    -0.7291519641876221f,     0.19925044476985931f,     0.2545233368873596f,     0.27186840772628784f,     -0.016324767842888832f,     0.10200028121471405f,     0.017517417669296265f,     -0.13640552759170532f, 
    -0.6445205807685852f,     0.24054808914661407f,     0.1441151201725006f,     0.14845456182956696f,     0.02364235371351242f,     -0.07840996980667114f,     0.04129726439714432f,     0.14861252903938293f, 
    0.09304922819137573f,     -0.07554358243942261f,     0.16193906962871552f,     -0.07804204523563385f,     -0.4774113893508911f,     0.16900040209293365f,     0.2746394872665405f,     0.053029417991638184f, 
    -0.2382870465517044f,     0.05766997113823891f,     -0.1165982335805893f,     -0.22362682223320007f,     -0.19975365698337555f,     0.25881528854370117f,     -0.17867156863212585f,     -0.20432089269161224f, 
    -0.13548718392848969f,     0.17969287931919098f,     -0.09629766643047333f,     -0.17197340726852417f,     -0.08914175629615784f,     -0.009898383170366287f,     -0.02796797640621662f,     -0.25166210532188416f, 
    -0.37958407402038574f,     0.16469375789165497f,     0.299041748046875f,     0.21002133190631866f,     -0.423592209815979f,     0.15102428197860718f,     0.012586493976414204f,     -0.10352969914674759f, 
    0.4642226994037628f,     -0.4868272840976715f,     0.16463154554367065f,     -0.2770529091358185f,     0.0687645748257637f,     -0.32790571451187134f,     0.38818761706352234f,     -0.3085705041885376f, 
    -0.004473534412682056f,     0.021590840071439743f,     -0.29477930068969727f,     -0.19188064336776733f,     -0.0813823863863945f,     -0.011444203555583954f,     0.05361393466591835f,     0.44211292266845703f, 
    -0.17022328078746796f,     0.0968819260597229f,     -0.21196883916854858f,     -0.19114947319030762f,     0.0724305510520935f,     0.18603725731372833f,     -0.09865564852952957f,     -0.215445876121521f, 
    0.38108620047569275f,     -0.2574753761291504f,     0.3627742826938629f,     0.0014420949155464768f,     -0.25986623764038086f,     0.5991880893707275f,     0.3511531949043274f,     -0.09163022041320801f, 
    -0.28648892045021057f,     0.5195803642272949f,     0.1699351668357849f,     0.4398081600666046f,     -0.5582802891731262f,     -0.13477723300457f,     0.1321296989917755f,     0.14684399962425232f, 
    0.005791160278022289f,     -0.25793421268463135f,     0.04640795290470123f,     -0.03177431598305702f,     0.20469756424427032f,     -0.012774978764355183f,     -0.010914579033851624f,     -0.35989150404930115f, 
    -0.27715805172920227f,     0.06875291466712952f,     0.023616690188646317f,     0.2658414840698242f,     -0.17465229332447052f,     0.17534784972667694f,     0.23566852509975433f,     0.1796247661113739f, 
    -0.5133809447288513f,     0.4481653571128845f,     0.1586645543575287f,     0.5019675493240356f,     -1.2672210931777954f,     0.5437454581260681f,     -0.02459978498518467f,     0.0991164892911911f, 
    -0.009284164756536484f,     -0.5745913982391357f,     0.3505566716194153f,     -0.40082108974456787f,     -0.13309346139431f,     -0.5445998907089233f,     0.1827690154314041f,     -0.37029561400413513f, 
    -0.165586456656456f,     0.03563109040260315f,     -0.08092691004276276f,     -0.10423863679170609f,     -0.042308252304792404f,     0.1117248684167862f,     -0.09926728904247284f,     -0.21868360042572021f, 
    -0.761573314666748f,     0.6252421140670776f,     0.2420961558818817f,     0.3858533799648285f,     -0.8630688190460205f,     0.6151718497276306f,     -0.04549431800842285f,     0.2934926748275757f, 
    -0.3305720090866089f,     -0.060484837740659714f,     -0.1954994946718216f,     0.36569854617118835f,     1.0835407972335815f,     0.30887916684150696f,     -0.6634542942047119f,     0.002916302066296339f, 
    0.42707541584968567f,     -0.5756070613861084f,     0.26629528403282166f,     -0.6220921874046326f,     -0.05809663236141205f,     -0.5594848394393921f,     0.2691422700881958f,     -0.3015044331550598f, 
    -0.32605990767478943f,     0.2505214512348175f,     0.23313747346401215f,     0.3560514748096466f,     -0.8442608714103699f,     0.1493009477853775f,     -0.05950009450316429f,     0.40872809290885925f, 
    0.15269124507904053f,     -0.2722356617450714f,     0.2087853103876114f,     -0.6121010184288025f,     -0.261468768119812f,     -0.6545656323432922f,     0.36361396312713623f,     -0.3706042468547821f, 
};

static const std::vector<float> LOUT_B = {
    0.03976477310061455f,     0.2139837145805359f,     0.1542692482471466f,     -0.003884314326569438f,     0.07631295919418335f,     0.09776514023542404f,     0.007272610440850258f,     0.023776374757289886f, 
    0.05737261846661568f,     0.13479933142662048f,     -0.14643406867980957f,     -0.00933600589632988f,     0.2090998888015747f,     0.05651386082172394f,     -0.13889220356941223f,     0.06027979031205177f, 
    -0.02690303511917591f,     0.22465239465236664f,     0.0060973502695560455f,     0.15458709001541138f,     -0.43563035130500793f,     0.1938435286283493f,     0.0985884815454483f,     0.2475804090499878f, 
};

static const std::vector<float> MEAN_W = {
    -0.1861787587404251f,     0.3485904932022095f,     0.22992056608200073f,     -0.34288641810417175f,     -0.27132460474967957f,     0.08628255873918533f,     0.0029765476938337088f,     -0.002879379317164421f, 
    -0.40650710463523865f,     0.4822182357311249f,     -0.5145776867866516f,     3.461504456936382e-05f,     0.09435679763555527f,     -0.2923668324947357f,     0.12077340483665466f,     -0.09508506953716278f, 
    -0.4394035339355469f,     0.3328094482421875f,     0.0005502699641510844f,     -0.49225395917892456f,     0.9004931449890137f,     0.5649946928024292f,     -0.14728060364723206f,     0.43258780241012573f, 
};

} // namespace NN

#endif // __NN_PARAMETERS_DEF_H__
