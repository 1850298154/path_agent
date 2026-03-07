# FuzzyVO论文: Fuzzy velocity obstacle for multi-robot navigation



--- Page 1 ---

Robotica(2023),41,pp.668–689
doi:10.1017/S0263574722001515
RESEARCHARTICLE
Cooperative collision avoidance in multirobot systems
using fuzzy rules and velocity obstacles
WenbingTang1 ,YuanZhou2,∗ ,TianweiZhang2,YangLiu2,JingLiu1andZuohuaDing3
1ShanghaiKeyLaboratoryofTrustworthyComputing,EastChinaNormalUniversity,Shanghai200062,China,2Schoolof
ComputerScienceandEngineering,NanyangTechnologicalUniversity,Singapore639798,Singapore,and3Schoolof
InformationScienceandTechnology,ZhejiangSci-TechUniversity,Hangzhou310018,China
∗Correspondingauthor.E-mail:y.zhou@ntu.edu.sg
Received:13June2022;Revised:22September2022;Accepted:26September2022;
Firstpublishedonline:28October2022
Keywords:collisionavoidance,fuzzyrules,multirobotsystems,velocityobstacles
Abstract
Collision avoidanceiscriticalinmultirobotsystems.Mostofthecurrentmethodsforcollisionavoidanceeither
requirehighcomputationcosts(e.g.,velocityobstaclesandmathematicaloptimization)orcannotalwaysprovide
safetyguarantees(e.g.,learning-basedmethods).Moreover,theycannotdealwithuncertainsensingdataandlin-
guistic requirements (e.g., the speed of a robot should not be large when it is near to other robots). Hence, to
guaranteereal-timecollisionavoidanceanddealwithlinguisticrequirements,adistributedandhybridmotionplan-
ningmethod,namedFuzzy-VO,isproposedformultirobotsystems.Itcontainstwobasiccomponents:fuzzyrules,
whichcandealwithlinguisticrequirementsandcomputemotionefficiently,andvelocityobstacles(VOs),which
cangeneratecollision-freemotioneffectively.TheFuzzy-VOappliesanintruderselectionmethodtomitigatethe
exponentialincreaseofthenumberoffuzzyrules.Indetail,atanytimeinstant,arobotcheckstherobotsthatitmay
collidewithandretrievesthemostdangerousrobotineachsectorbasedonthepredictedcollisiontime;then,the
robotgeneratesitsvelocityinreal-timeviafuzzyinferenceandVO-basedfine-tuning.Ateachtimeinstant,arobot
onlyneedstoretrieveitsneighbors’currentpositionsandvelocities,sothemethodisfullydistributed.Extensive
simulationswithadifferentnumberofrobotsarecarriedouttocomparetheperformanceofFuzzy-VOwiththe
conventionalfuzzyrulemethodandtheVO-basedmethodfromdifferentaspects.Theresultsshowthat:Compared
with the conventional fuzzy rule method, the average success rate of the proposed method can be increased by
306.5%;comparedwiththeVO-basedmethod,theaverageone-stepdecisiontimeisreducedby740.9%.
1. Introduction
Amultirobotsystemisasystemcontainingmultiplerobots,suchasunmannedaerialvehicles(UAVs)
andunmannedgroundvehicles(UGVs),thataremovingaroundinagivenenvironmenttoaccomplish
tasks cooperatively. Compared with their single-robot counterparts, multirobot systems can increase
functionalities,improveefficiency,enhanceadaptability,andproviderobustness[1–3].Multirobotsys-
temshavebeenappliedtodealwithlabor-consumingordangerousmissions,suchasassembly,disaster
rescue,environmentalprotection,trafficmonitoring,militaryreconnaissance,cargodelivery,andmany
otherfields[4–6].
Coordinated motion is one of the most important requirements in a multirobot system. However,
due tothe complexity of the environment and thesimultaneous motion of robots,collisions are com-
monincoordinatedmotion.Manymethodshavebeenproposedtoavoidcollisionsduringrobotmotion.
They can be mainly classified into two categories: model-driven methods and data-driven methods.
Model-drivenmethods,suchasformalmethods[7,8],discreteeventsystemmethods[9–11],potential
field methods [12], velocity obstacles (VOs) [13, 14], model predictive control [15], and mathemati-
caloptimizationmethods[16],relyonthemodelsofrobotsand/orenvironments.Specifically,formal
(cid:3)CTheAuthor(s),2022.PublishedbyCambridgeUniversityPress.
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 2 ---

Robotica 669
TableI. Summaryofdifferentcollisionavoidancemethods.
Methods Description Advantages Disadvantages
Model-driven Formalmethods Describemotion Safetyguarantee Limitedto
methods requirements structurized
usingLTLand/or environments
CTL
Potentialfield Buildattractiveand Unknown Localminima,
methods repulsive environment high
potential complexity
functions
Velocity Constructavelocity Movingobstacles, Highcomputation
obstacles obstacleinthe safety cost,
velocityspace guarantee oscillatory
motion
Modelpredictive Buildapredictive Flexibility Highcomputation
control modelofthe cost
controlsystem
Mathematical Constructaproper Modelingmultiple Highcomputation
optimization optimization constraints cost
methods problemand
solveit
Data-driven Fuzzyrules Buildafuzzyrule Uncertainty, Oscillatingpaths,
methods baseandselecta real-time poor
properinference inference generalization
mechanism ability
Swarm Defineaproper Fastgenerationof Localoptimum,
intelligence optimization acceptable unexpected
algorithms goalandstrategy solutions solutions
Deep Learntomaximize Unstructureddata, Hightrainingcost,
reinforcement theexpected dynamic lowsampling
learning cumulative environment efficiency
reward
methodsapplythetechnologiessuchasformalverificationandmodelcheckingtocontrolrobots’motion
[7,8].Discreteeventsystemmethodsapplysupervisorycontroltheorytoavoidcollisionsanddeadlocks
formultirobotsystems[9–11].Potentialfieldmethodsdefineproperattractivepotentialfunctionsand
repulsivepotentialfunctionstoleadarobottoitstargetwhileavoidingobstacles[12].VO-basedmeth-
odscomputeacollision-freevelocityfromtheunionvelocityspaceofallobstaclesateachtimeinstant
[13]. Model predictive control applies an explicit model to describe the control system and obtains a
sequenceofcontrolinputsbysolvinganoptimizationproblembasedonthemodel[15].Mathematical
optimizationmethodsgeneratecontrolactionsbymodelingthecollisionavoidanceproblemasanopti-
mization problem [16]. Data-driven methods are learning-based methods using sample data to learn
proper controllers, such as fuzzy rules [15], swarm intelligence algorithms [17], and deep reinforce-
mentlearning(DRL)[18,19].Detailedly,fuzzyrulesgenerateanactionviafuzzyinferenceoftherules
extractedfromthecollecteddata[15].Swarmintelligencealgorithmsiterativelysearchforactionsinthe
regiondefinedbythepreviousoptimalmovementsoftherobotanditsneighbors[17]. DRLformalizes
thecollisionavoidanceproblemasaMarkovdecisionprocess,whichissolvedbylearningadecision
policymappingfromthestatespacetotheactionspace[18,19].TableIgivesabriefsummaryofeach
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 3 ---

670 WenbingTangetal.
methodanditsadvantagesanddisadvantages.Eventhoughcollisionavoidancehasbeenwidelystudied,
therearesomechallengingproblemsthatarenotadequatelyaddressed.First,mostofthemodel-driven
methodscanguaranteesafetybutrequirehighcomputationcosts.Second,data-drivenmethodsleverage
offlinelearningtoimproveonlinecomputationefficiencybutcannotprovidecollision-freeguarantees.
Inthispaper,anewreal-timecollisionavoidanceapproach,namedFuzzy-VO,isproposedtoguar-
anteesafetyandcomputationefficiencyformultirobotsystemswithsensinguncertaintyandlinguistic
data. It combines fuzzy rules and VOs. On the one hand, considering uncertainties in sensing data,
linguistic requirements may exist for a robot’s motion, for example, moving slowly and turning right
slightly.Fuzzyrulesareawell-establishedtoolto(1)dealwithnotonlycrispdatabutalsouncertain
andlinguisticdataand(2)expressthebehaviorofasysteminaninterpretableway.However,mostofthe
currentfuzzyrule-basedmethodsareforasinglerobot[20,21].Thedesignoffuzzyrulesforcollision
avoidanceinamultirobotsystemisstillchallengingsince:(1)theformofafuzzyruleisdependenton
thenumberofrobots;(2)thenumberoffuzzyrulesincreasesexponentiallywiththenumberofrobots
[22]; and (3) a robot’s motion may oscillate because of the large number of rules (the claim will be
empiricallyvalidatedinexperiments).Ontheotherhand,VO-basedmethods,suchasoptimalrecipro-
calcollisionavoidance(ORCA)[14],arewelldefinedandgenerallyapplicabletechniquesforreactive
obstacleavoidancewiththeexistenceofdynamicobstacles[13].However,computinganoptimalveloc-
ity for a robot in a multirobot system is still challenging since: (1) the scale of the problem (e.g., the
numberofconstraints)increaseswiththenumberofrobots,andsodoesthecomputationcostand(2)
therobotmayhavefewvelocitycandidatesifnumerousobstaclesarearound.
Tomitigatetheabovedrawbacks,Fuzzy-VOfirstusesaunifiedintruderselectionmethodtogenerate
fuzzyruleswithanarbitrarynumberofrobots.Specifically,givenarobot,Fuzzy-VOdividesitssensing
regionintoafixednumberofsectors.Ineachsector,therobotappliestheVOtechnologytoevaluate
and select the most dangerous robot to perform collision avoidance. Hence, the maximal number of
robots to be avoided by a robot is constant. In this way, Fuzzy-VO can determine the form and the
numberoffuzzyrules.Then,sampledataarecollectedviaORCAtolearnafuzzyrulebase.Thus,the
robot can compute the candidate motion in realtime via fuzzy inference on the rule base. Second, to
guaranteethatthefinalmotioniscollision-free,Fuzzy-VOappliestheVOtechnologytocheckandfine-
tune,ifnecessary,thecandidatemotion.Sinceeachrobotonlyneedstoretrievethecurrentstatesofits
neighbors,whichcanbeobtainedimmediately,Fuzzy-VOisfullydistributed.Thesameright-of-way,
suchastheturn-rightrule,isappliedtoguaranteemutualexclusionduringdistributeddecisionmaking.
AsetofsimulationsarecarriedoutwithmultipleUAVs.Theresultsdemonstratetheeffectivenessof
Fuzzy-VO in addressing potential collisions. Extensive comparison results show that Fuzzy-VO can
reduce the number of rules and generate smoother paths compared with the conventional fuzzy rule
approachandimprovecomputationefficiencycomparedwiththeORCAmethod.
Themaincontributionsofthispaperarethreefold:
1. Basedonsensingregionpartitionandintruderselection,apracticalstrategyisproposedtobuild
afuzzyrulebasewithanarbitrarynumberofrobots.
2. For each robot, a strategy is developed to generate collision-free velocities based on fuzzy
ruleinferenceandVO-basedfine-tuning.Itleveragesthecomputationefficiencyofdata-driven
methodsandthesafetyguaranteeofmodel-drivenmethods.
3. Basedontheabovetwostrategies,afullydistributedandreal-timecollisionavoidancemethod,
Fuzzy-VO,isproposedformultirobotsystemswithanarbitrarynumberofrobots.
Therestofthispaperisorganizedasfollows.Section2summarizestherelatedwork.Section3states
theproblemsolvedinthispaper.Section4givesanoverviewofFuzzy-VO.Sections5and6presentthe
proceduresforintruderselectionandcollision-freevelocitygeneration,respectively.Section7provides
the detailed algorithms, as well as the complexity analysis. Simulations are conducted in Section 8
to demonstrate the effectiveness and efficiency of Fuzzy-VO. Conclusion and future work are finally
providedinSection9.
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 4 ---

Robotica 671
2. Relatedwork
This paper is related to the topic of collision avoidance, which is a key and popular topic in
robotics. Many methods have been proposed in this area. For example, Wang et al. [23] proposed
a three-dimensional navigation strategy for nonholonomic robots with moving obstacles, where the
robot’smotiondirectionmaintainsaconstantanglewiththeobstacles’boundarytangentstoavoidcolli-
sions.Lindqvistetal.[15]formalizedthecollisionavoidanceproblemasanoptimizationprobleminthe
frameworkofmodelpredictivecontrolandresolveditusingOpEnsolver.In[24],apotentialfield-based
collisionavoidanceapproachwasproposedfornonholonomicUAVs,whichtookthevelocitydirection
ofanobstacleintoconsiderationduringthedesignofpotentialfieldfunctions.
Among the existing approaches, fuzzy rules are a promising tool to deal with data uncertainties
and linguistic requirements for multirobot systems. Llorca et al. [25] proposed a fuzzy control-based
autonomouscollisionavoidancesystem.Inthissystem,thelateraldisplacementandtheactualspeedof
thevehicleareusedasfuzzyinputs,andtheoutputofthefuzzysteeringcontrolleristhesteering-wheel
position.Vadakkepatetal.[26]proposedafuzzybehavior-basedarchitectureforthecontrolofmobile
robotsinamultiagentenvironment.Inref.[20],afuzzyobstacleavoidancecontrollerisproposedfor
an autonomous vehicle using both negative fuzzy rules and traditional positive rules. The proposed
architecturecanbedecomposedintofourrobotroles,12robotbehaviors,and14robotactions,where
obstacleavoidanceisfulfilledbyindependentbehaviors.However,themajordrawbackoftheseworksis
thegeneralizationability,thatis,theyareunsuitableforscenarioswithavariablenumberofrobots.Wen
etal.[21]dividedthesensingregionsoftheultrasonicsensorsonaUAVintothreegroups:front,left,
andright,andthentookeachgroup’sminimumobstacledistanceastheinputsofitsfuzzycontroller.
Changetal.[27]proposedatwo-layerfuzzylogiccontrollerformultirobotcoordination,whichdivides
thescanningareaintosevensectorsandselectstheshortestdistancetoadetectedobstacleineachsector
asinput.Buttheselectionmaymissobstaclesthatarethreateningandemergentineachsector.
VO-basedmethodsareanotherkindofpromisingmethodsforcollisionavoidance.VOisfirstpro-
posed by Fiorini and Shiller [28]. It is a velocity-based approach to avoiding collisions with moving
obstacles. In VO, the velocity space of a robot is divided into collision and collision-free velocities,
and an appropriate collision-free velocity is computed at any time instant. However, VO suffers from
someweaknessessuchasundesirableoscillatorymotionandreciprocaldances[29,30].Someimproved
variations, such as reciprocal VO [13] and ORCA [14], have been proposed. van den Berg et al. [14]
proposed the sufficient condition for multiple robots to avoid collisions and guarantee collision-free
motion.Bysolvingalinearprogram,eachrobotselectsitsoptimalvelocityfromtheintersectionofall
possible half-planes in the velocity space. Jenie et al. [31] proposed a cooperative autonomous colli-
sion avoidance algorithm named selective velocity obstacle (SVO), which is also an extension of the
originalVO.Especially,whenSVOneedstoavoidapossiblecollisionaccordingtothedetection,the
right-of-way rules for manned flight are taken into account in the decision-making process. Recently,
Hanetal.[19]combinedVOwithDRLtodealwiththereciprocalcollisionavoidanceproblemunder
limitedinformationscenarios.Inref.[32],VOisappliedtodeducethecollisionconditionsinconnected
andautomatedvehicles.However,theseVO-basedmethodsrequirehighcomputationcostsincrowded
environments[33].
Comparedwiththeaforementionedmethodsintheliterature,themethodproposedinthispaperaims
toachievereal-timecomputationefficiencyandsafetyguaranteessimultaneouslyformultirobotsystems.
3. Problemstatement
Thescopeofthispaperisthecooperativemotionofasetofrobotsmovingina2Dspace,forexample,a
setofUGVsmovingonthegroundormultipleUAVsmovingatthesameheight.Assumethatthereare
N holonomicrobotsmovinginthesameenvironment.Fornonholonomicrobots,itisrecommendedto
readrefs.[23,24]formoredetails.Notethattoperceivetheenvironment,eachrobotisequippedwith
differentsensors,forexample,camerasandLiDARs.However,duetomeasurementerrors,uncertainties
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 5 ---

672 WenbingTangetal.
Figure 1. Robot r with two intruders r and r (i.e., the three circles) in the body frame C. The
i i1 i2 i
semicircleareaisthecollisionregionoftherobotr.
i
mayexistinthesensingdata.Themotiontaskforarobotr istomovefromitsinitialpositionp0∈R2
i i
to the target position pf ∈R2 within a given duration τ, where R2 is the 2D Euclidian space. Each
i
robot r has a reference speed vref, where vref ≤v , and v is the upper bound of the robot speed.
i i i max max
Moreover,itisassumedthateachrobotcannotmovebackward.Sincemultiplerobotsaremovinginthe
same environment, different robots need to avoid collisions with each other. In this work, each robot
is modeled as a sphere, and its position is identified by its center. Each robot regards other robots as
dynamicobstacles.Forsimplicity,otherdynamicobstaclesarenotconsidered.
Beforegivingtheproblemstatement,somesymbolsanddefinitionsaredefined.Givenarobotr,its
i
positionandvelocityattimetaredenotedasp(t)andv(t),respectively.Clearly,∀t>0,(cid:7)v(t)(cid:7)≤v .
i i i max
Sincethemotionisassumedina2Dspace,p(t)∈R2andv(t)∈R2.Thestateofarobot,denotedass,is
avectorcontainingtherobot’spositionandvelocity,thatis,s=(p,v).Thesetofallpossiblestatesof
arobotr isdenotedasS.Thetrajectoryofarobotr,denotedasTr,isatime-parameterizedfunction
i i i i
mapping from R+ to S, that is, Tr(t)=s(t)=(p(t),v(t))∈S. By discretizing the time into discrete
i i i i i i
timeinstantswiththesametimestep,thatis,0=t ,t ,...,t =τ,themotionofr canbeformalized
0 1 K i
as
p(t)=p(t )+v(t )(t−t ), t∈[t ,t )
i i k i k k k k+1
p(t )=p0, p(t )=pf,
i 0 i i K i
v(t )=[v(t )cosθ(t ),v(t )sinθ(t )]. (1)
i k i k i k i k i k
wherev(t )∈Randθ(t )∈Rarethespeedandmotiondirectionofrobotr att ,respectively.
i k i k i k
Atanytimeinstant,arobotneedstomonitoraproperregion,denotedascollisionregion,withrespect
toitscurrentposition.Thebodyframeofarobotr isdenotedasC.ItisaCartesiancoordinatesystem
i i
whose origin is the center of r, the y-axis is the same as v, and the x-axis is perpendicular to the
i i
y-axis. As shown in Fig. 1, at the current instant, r is at O, and its velocity is v. Then the related
i i
C isXOY,wheretheY-axisisv.Sinceeachrobotcannotmovebackward,thepossiblemotionareais
i i
{(x,y)|y≥0}.Hence,thecollisionregionofarobotr attimetcanbedefinedasCR(t)={(x,y)∈C|0≤
i i i
x≤Lcos(θ),0≤y≤Lsin(θ),0≤θ≤π},providedthatthesensingrangeisL.Toguaranteesafety,each
robotneedstoavoidcollisionswithotherrobotsinitscollisionregion.Atanytimet,tworobotsr and
i
r areinacollisionif(cid:7)p(t)−p(t)(cid:7) <2ρ,wherep(t)∈S,p(t)∈S,andρ isthesaferadiusforeach
j i j 2 i i j j
robot.Hence,thereisthefollowingdefinition.
Definition1(Intruder).Arobotr iscalledanintruderofr attimetifp(t)∈CR(t)and∃t(cid:10)∈(t,τ)such
j i j i
that(cid:7)p(t(cid:10))−p(t(cid:10))(cid:7) <2ρ,wherep(t(cid:10))=p(t)+v(t)(t(cid:10)−t)andp(t(cid:10))=p(t)+v(t)(t(cid:10)−t).
i j 2 i i i j j j
For example, as shown in Fig. 1, the three robots, r, r , and r , are moving to pf, pf , and pf ,
i i1 i2 i i1 i2
respectively. At the current time, r detects that r and r will collide with it if all of them keep their
i i1 i2
currentvelocities,sor andr areintrudersofr.
i1 i2 i
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 6 ---

Robotica 673
Figure2. FrameworkofFuzzy-VO.
Basedontheabovedescriptions,togenerateacollision-freetrajectoryforarobot,therobotneeds
todetermineitsvelocityatthediscretetimeinstants.Hence,theproblemstudiedinthispapercanbe
describedasfollows:
Problem1.Givenasetofrobots,eachofwhichmaycontainuncertainsensingdataduringitsmotion,
decidethemotionofeachrobot,thatis,itsvelocityvectors,suchthattherobotcanalwaysdetectand
avoidpotentialcollisionswithitsintruders.
4. Overviewoftheproposedmethod
Todealwithuncertaintiesinsensingdataandguaranteereal-timeefficiency,aVO-aidedfuzzyinference
methodisproposedtogeneratecollision-freemotionforeachrobot.Thissectiongivestheframework
oftheproposedFuzzy-VO,whilethedetailsaregiveninthefollowingsections.
Figure2showsthehigh-levelworkflowofFuzzy-VO.Themainideaistorestrictthenumberoffuzzy
rulesbyselectingaproperandfixednumberofintrudersforcollisionavoidanceratherthanconsidering
allintruders.Inthisway,Fuzzy-VOcanbeadoptedtoadifferentnumberofrobotsandguaranteeflexi-
bilityandscalability.Itmainlycontainsthreeprocesses,thatis,intruderselection,fuzzyrulegeneration
andinference,andvelocitygeneration.
Intruderselection.Foreachrobot,thefirststepistopartitionitscollisionregionintoasetofdisjoint
sectors.Asectormaycontainseveralintruders,anddifferentsectorsmaycontainadifferentnumberof
intruders.Hence,togenerateauniversalmethodforadifferentnumberofrobots,apropernumberof
intrudersshouldbeselectedineachsector.Inthispaper,onlythemostdangerousintruderisselected
foreachsectorbasedonthetechnologyofVOs.
Fuzzy rule generation and inference. Fuzzy rule-based collision avoidance technology is used to
generateacandidatemotioncommand.First,tobuildthefuzzybase,theconventionalmotionplanning
algorithms are applied to generate corresponding sampling data. Then, fuzzy rules can be extracted
fromthecollecteddata.Whentherulebaseisconstructed,atanyinstant,therobotcanperformfuzzy
inferencebasedonthecurrentselectedintrudersandgenerateacandidatevelocity.
Velocity generation. Since the fuzzy rules are generated based on sampling data rather than exact
system models, the reasoning results cannot always guarantee collision avoidance. Hence, validation
of the candidate velocity is required to generate the actual velocity. In this paper, it is assumed that
eachrobotmovestoitsrightsidetoavoidcollisions.Hence,therobotonlyneedstocheckwhetherthe
generatedvelocityisintheVOsoftherightintrudersandfine-tunesthecandidatevelocityifneeded.
5. Intruderdeterminationandselection
Thissectiondescribesthemethodtoselectapropernumberofintrudersforcollisionavoidancebythe
robot. Specifically, the partition of the collision region is first introduced, followed by the VO-based
intruderselectionmethodtoretrievethemostdangerousintruderineachpartition.
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 7 ---

674 WenbingTangetal.
Figure 3. Partition the collision region into three sectors and screen the most dangerous intruder in
eachsector.
5.1. Collisionregionpartition
At any time instant, r’s collision region CR(t) is partitioned into l equal sectors, that is,
i i
l ={(x,y)|0≤x≤Lcos(θ),0≤y≤Lsin(θ),(i−1)π/l≤θ≤iπ/l}, i=1,2,...,l. Consequently, an
i
appropriatevalueoflshouldbeselected.Consideringthatmanyreal-worldmobilerobotsareequipped
withthreegroupsofsensorsinthefront,suchas[34,35],inthispaper,thecollisionregionisequally
partitioned into three sectors: the left region (LR), the front region (FR), and the right region (RR).
TherobotsinLRareleftintruders,thoseinFRarefrontintruders,andinRRarerightintruders.For
example,Fig.3showsthethreesectorsofr,whereeachsectorhasacentralangleofπ/3.r andr are
i i1 i2
inLR,r isinFR,andr andr areinRR.Theyaretheintrudersofr atthecurrentinstant.
i3 i4 i5 i
5.2. VO-basedintruderselection
To avoid the explosion of the number of fuzzy rules with the number of intruders, in this subsection,
aVO-basedmethodisproposedtoselectaproperintruderineachsectorsuchthatthegeneratedrules
withafinitenumber,independentofthenumberofrobots.
The main idea of VO is to select a velocity of a robot outside the VO, which is the set of velocity
thatmaycausecollisionswithotherrobotsorobstacles.AsshowninFig.4,supposerobotr currently
i
isatp.Itdetectsanintruderr initscollisionregionCR,whosepositionandvelocityarep andv ,
i i1 i i1 i1
respectively.Sor shouldselectavelocityv toavoidcollisionwithr .Atthecurrenttimeinstant,r’s
i i i1 i
collisionregionwithrespecttor canbedescribedasC ={p(i)∈R2|(cid:7)p(i)−p (cid:7) <2ρ},thatis,the
i1 i|i1 i1 2
regionwithinthedashedcircleinFig.4.Therelativevelocityofr withrespecttor canbedescribedas
i i1
v =v −v .Therelativemotioncanbedefinedasλ(p,v )={p +tv |t>0},thatis,theblueray
i|i1 i i1 i i|i1 i i|i1
inFig.4.Clearly,r andr willcollideinthefutureifλ(p,v )∩C (cid:12)=∅.Hence,theVOofr related
i i1 i i|i1 i|i1 i
tor isdefinedasVO ={v|λ(p,v )∩C (cid:12)=∅},thatis,thegrayconeinFig.4.
i1 i|i1 i i i|i1 i|i1
BasedonVO,amethodisproposedtoselectthemostdangerousintruderineachsectionaccording
to the collision risk, which is defined as the potential collision time. Furthermore, the collision time
criterionisdefinedtoevaluatethecollisionrisk.
Definition 2(Potential Collision Time). The potential collision time of r with respect to r, denoted
i j
as(cid:7)Tc(j),isthe estimated shortesttimeduration fromthecurrenttime instanttothe occurrence of a
i
potentialcollisionbetweenr andr withtheircurrentvelocities.
i j
In detail, the computation of potential collision time is as follows. Consider the relative motion of
r with respect to r. As shown in Fig. 5, for the relative motion, r is with zero velocity, and r has a
i j j i
relative velocity v(t)−v(t). Clearly, the minimum distance is reached at the time when the relative
i j
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 8 ---

Robotica 675
Figure 4. Illustration of VO. VO is the velocity obstacle of r. Each velocity in VO will cause a
i|i1 i i|i1
collisionwithr insometimeinstantofthefuture.
i1
Figure5. Therelativemotionofr withrespecttor.
i j
motionreachespositionA.Thedistancefromp toAis
i
(p(t)−p(t))T(v(t)−v(t))
d(p,A)= j i i j (2)
i (cid:7)v(t)−v(t)(cid:7)
i j 2
d(p,A)
Hence,theestimatedtimefromp toAis i ,thatis,
i (cid:7)v(t)−v(t)(cid:7)
i j 2
(p(t)−p(t))T(v(t)−v(t))
(cid:7)T(j)= j i i j . (3)
i (cid:7)v(t)−v(t)(cid:7)2
i j 2
BasedontheprocedureofVO,ifv ∈VO ,d(j)<2ρ,andviceversa.Inthiscase,acollisionbetween
i i|j i
r andr happenswhentherelativemotionarrivesatpositionB,asshowninFig.5.Hence,(cid:7)Tc(j)can
i j i
becomputedasfollows:
(cid:7)Tc(j)=(cid:7)T(j)−t(B,A)
i i
(cid:2)
4ρ2−d2(j)
=(cid:7)T(j)− i . (4)
i (cid:7)v(t)−v(t)(cid:7)
i j 2
Clearly, the smaller (cid:7)Tc(j) is, the more dangerous r is, and the higher priority it has for collision
i j
avoidance. According to the estimated collision time, the most dangerous intruder is selected in each
sector,thatis,theintruderwiththesmallest(cid:7)Tc(j)ineachsector.Forexample,asshowninFig.3,the
i
selectedintruderinLRisr asithassmallercollisiontimethanr .
i2 i1
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 9 ---

676 WenbingTangetal.
6. Collisionavoidanceusingfuzzyrulesandvelocityobstacles
In this section, following the collision region partition, the process to build a fuzzy rule base is first
presented,andthentheprocedureforvelocitygenerationisprovided.
6.1. Introductionoffuzzyrules
Thissubsectionfirstgivesabriefintroductionoffuzzyrules,andthedetailscanbefoundinref.[36].
LetUbethedomainofdiscourseandu∈U.AfuzzysetφinUischaracterizedbyareal-valuefunction
μ :U→[0,1],whichassignseachelementinUwitharealnumberintheinterval[0,1].
φ
AsinglefuzzyIF-THENrule(orsimplyfuzzyrule)isdefinedonlinguisticvariableswiththeform:
IFxisA(premise)THENyisB(consequence)
wherexandyaretwofuzzy/linguisticvariables.AandBaretwofuzzysets.Amoregeneraltypeof
fuzzyrulesinpracticecanbedescribedas:
IFx isA ∧···∧x isA THENy isB ∧···∧y isB ,
1 1 p p 1 1 q q
wherep≥1andq≥1areintegers.
6.2. Fuzzyrulegeneration
Ononehand,accordingtotheselectionofintruders,theinputofafuzzyruleisthecollisiontime(cid:7)Tc
of the selected intruders. On the other hand, since the kinematic model of each robot considered in
this paper is unicycle kinematics, the output variables of fuzzy rules are set as the speed ratio α and
the orientation change (cid:7)θ. The new speed is v(cid:10) =αv , which adjusts the current speed v according
c c c
to a proper ratio α. Note that, to guarantee mutual exclusion during distributed decision making and
avoid collisions, each robot is expected to turn right with a proper direction, so (cid:7)θ∈[0,π/2], and
the new orientation is θ −(cid:7)θ, where θ is the current orientation of r. Then, the selected velocity is
i i i
v(cid:10)=(v(cid:10) cos(θ −(cid:7)θ),v(cid:10) sin(θ −(cid:7)θ)).Notethattheoutputsoffuzzyrulesaredeterminedbytherobot
i c i c i
kinematicsdescribedin(1),ratherthanrobotdynamics,suchasinertia.
Inthesequel,thegenerationoffuzzyrulesisdescribed,thatis,fuzzification,datasampling,andrule
determination.
6.2.1.Fuzzification
The main task of fuzzification is to translate crisp variables into the corresponding linguistic ones.
Hence,foreachcrispvariablex,thesetoffuzzytermsandtheircorrespondingmembershipfunctions
shouldbedetermined.Userscanselectanymembershipfunctionaslongasitcanmapthecrispdatainto
desired degree of memberships. In this paper, the triangular membership function is applied for each
fuzzy set as (1) it has been proven to have good quality results and computational efficiency in many
practicalapplications(includingrobotmotioncontrol)[37,38]and(2)itshowsgoodperformancein
oursimulationexperiments.
First, consider the fuzzification process of the input variable (cid:7)Tc. Usually, a robot needs a time
duration,sayresponsetime,toperformcollisionavoidance,includingcollisionpredictionanddecision
execution.Basedontheconfigurationsofarobotanditshistorymotionrecords,theminimalandmax-
imal response time of the robots can be determined, denoted as t and t , respectively. If (cid:7)Tc is less
1 2
than t , then the situation is very emergent, and the robot needs to perform some special actions, for
1
example,stopimmediately,toavoidcollisions.Otherwise,if(cid:7)Tcissmall,meaningthattheremaining
timetotakecollisionavoidanceactionsisshort,thenthissituationisdangerous,andtherobotneedsto
doitsbesttoavoidcollisions;whileif(cid:7)Tc islarge,meaningthattherobothasenoughtimetoavoid
collisions,thenthecurrentmotionissafe.Hence,threefuzzysets,thatis,E(emergent),D(dangerous),
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 10 ---

Robotica 677
(a) Collision time. (b) Speed ratio. (c) Orientation change.
Figure 6. Membership functions for all variables. (a) Membership function of (cid:7)Tc; (b) membership
functionofα;(c)membershipfunctionof(cid:7)θ.
(a) An intruder is in LR. (b) An intruder is in FR. (c) An intruder is in RR.
Figure7. Scenariosthatthereisonlyonethreadinasector.
andS(safe),aredefinedtodescribe(cid:7)Tc.TheirmembershipfunctionsareshowninFig.6(a).Moreover,
theformaldefinitionsofallmembershipfunctionsaregivenintheappendix.
Second,considerthefuzzificationoftheoutputvariables.(1)Thespeedvariableispartitionedinto
fourfuzzysets:MA(maintain),DS(decelerateslightly),DL(deceleratelargely),andSU(stopurgently).
Sincethelevelofspeeddecelerationisrelatedtothecurrentspeed,theratioofthenewspeed(v(cid:10))tothe
c
currentspeed(v ),denotedasα,istheinputsofthethreemembershipfunctions.Notethat0≤α≤1.The
c
graphicalrepresentationofthesemembershipfunctionsisgiveninFig.6(b),whereα isthethreshold
0
ofamaintainingactionandcanbedeterminedbyanexpertorbasedonusers’requirements.(2)Forthe
changeoforientation,fivefuzzysetsareappliedtodescribeitsvalues:VS(verysmall),SM(small),M
(medium),L(large),andVL(verylarge).TheirmembershipfunctionsareshowninFig.6(c).Notethat
onecandefinemorefuzzysetsforspeedandorientationchangeifneeded.
6.2.2.Buildingoffuzzyrules
As described before, to avoid an exponential increase in the number of fuzzy rules, a robot selects at
mostoneintruderineachsectortoperformcollisionavoidance.Hence,thepremiseofarulecontainsat
mostthreefuzzypropositions,eachofwhichhasfourpossibleforms,includingtheemptysituation.So
thereare43−1=63possiblepremises.Theconsequenceofarulecontainsatmosttwoindependent
fuzzypropositions,andtheyhavefourandfivepossibleforms,respectively.Hence,thenumberofcan-
didaterulesis63∗(4+5)=567.However,properrulesshouldbeselectedfromthecandidates.Foreach
premise, the next step is to determine the consequences related to the two output linguistic variables,
respectively.Theruleselectionisbasedonsupportingdegreesdescribedinref.[39].Foreachpremise,
thesupportingdegreesofallcandidaterulesarecomputed,andonlytherulewiththemaximaloneis
selected.Notethatruleswhosesupportingdegreesarezeroarealsofiltered.
Basedonthepartitionofthecollisionregionandintruderselection,therearethreekindsofscenarios
during the generation of fuzzy rules. The first one is that there is only one sector existing a selected
intruder,asshowninFig.7.ForeachoneshowninFig.7,thesamplingdataaregeneratedfrommultiple
simulationrunsbysettingdifferentstatusoftheintruder.Foreachrun,theORCAalgorithmisperformed
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 11 ---

678 WenbingTangetal.
(a) In LR and FR. (b) In FR and RR. (c) In LR and RR. (d) In LR, FR, and RR.
Figure8. Scenarioswithmultipleintruders.
togeneratethecooperativemotiondatabetweenr andtheintruder.Forexample,supposeattimeinstant
i
t,therobotr,withthestate(p(t),v(t)),detectsanintruderr ,whosestateis(p (t),v (t)),inLR,and
i i i i1 i1 i1
theORCAmethodgeneratesanewvelocityv (t)forr.Letθ (t)andθ (t)denotetheorientation
i,ORCA i vi vi,ORCA
angleofv(t)andv (t)intheinertialframeXOY.Whenθ (t)≤θ (t)and(cid:7)v (t)(cid:7) ≤(cid:7)v(t)(cid:7) ,
i i,ORCA vi,ORCA vi i,ORCA 2 i 2
asample(L ,(cid:7)T(i1),α(t),(cid:7)θ(t))iscollected,where(cid:7)T(i1)iscomputedbasedon(4),whileα(t)
(cid:7)Tc i i i i i
and(cid:7)θ(t)arecomputedbasedonthefollowingequations.
i
(cid:7)v (t)(cid:7)
α(t)= i,ORCA 2, (5)
i (cid:7)v(t)(cid:7)
i 2
(cid:7)θ(t)=θ (t)−θ (t) (6)
i vi vi,ORCA
In this way, a total of 1664 valid records are generated from the first kind of scenarios. With
all the records, six rules are generated, whose details are given in the appendix and the website
https://fuzzyvo.github.io/. Note that the original rules are in the form with single-input–single-output,
suchas“IFL isD,THENv isDS”.However,theruleswiththesamepremisecanbecombined,for
(cid:7)Tc c
example,fortherules“IFL isD,THENv isDS”and“IFL isD,THENv isL”,theycanbe
(cid:7)Tc c (cid:7)Tc c
combinedasone“IFL isD,THENv isDSand(cid:7)θ isL”.
(cid:7)Tc c
The second kind of scenarios is that there exist two sectors such that either of them contains an
intruder,asshowninFig.8(a)–(c).Similarly,29,347samplesarecollectedinthiskindofscenarios,and
12fuzzyrulesaregenerated,whicharegivenintheappendix.Thethirdoneisthateachsectorcontains
aselectedintruder,asshowninFig.8(d).Togenerateproperrulesforthiskindofscenarios,atotalof
5299recordsarecollectedtotrainrules.
Whenthecollisiontimerelatedtotheintruderinasectorisemergent,therobotshouldstopurgently.
Hence, threeemergent rulesarealsointroduced, which arealsogiven intheappendix. Finally,arule
basecontaining29fuzzyrulesisbuilt.Allgeneratedfuzzyrulesarelistedintheappendix.
6.3. Velocitygenerationviafuzzyinference
Whentherulebaseisbuilt,thevelocitiescanbegenerateddirectlyviafuzzyinference.Inthispaper,the
Mamdani(min)inferencemechanismisapplied,whoseoutputisafuzzyset[40].Themechanismcanbe
describedasfollows.Givenaninputx0=(x0,...,x0)andtheactivatedfuzzyrule“rule :IFx isA ∧
1 p 1 1 1
···∧x isA THENyisB,”theinferentialfuzzysetfortheconsequenceiscomputedbasedon(7).
p p
inf(x0,rule )=A (x0)∧...∧A (x0)
1 1 1 p p
B(y,x0,rule )=inf(rule )∧B(y) (7)
1 1
where A (x0)∧...∧A (x0)=min{A (x0),...,A (x0)}. For the activation of multiple rules, the final
1 1 p p 1 1 p p
inferentialfuzzysetisdeterminedbasedon(8).
B(y,x0)=B(y,x0,rule )∨...∨B(y,x0,rule )
i1 ij
=max{B(y,x0,rule ),...,B(y,x0,rule )} (8)
i1 ij
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 12 ---

Robotica 679
(a) (b)
Figure9. Illustrationofvelocityrefinement.vg isobtainedfromfuzzyinference,vf isrefinedvelocity.
i i
(a)p isoutofVO .(b)p isinVO .
i i|i1 i i|i1
SinceB(y,x0)isafuzzyset,defuzzificationisrequiredtogenerateacrispvalueofthevelocityfor
therobot.Oneofthecommondefuzzificationmethodsisthecenterofgravity,whichdeterminesacrisp
valuebasedonthecenterofgravityofthegeneratedfuzzyset[41].Thecomputationofthecenterof
gravityisgivenin(9).
(cid:3)
yB(y,x0)dy
y∗= (cid:3) (9)
B(y,x0)dy
Based on the above procedure, given the current states of a robot and its intruders, the collision
times related to different intruders are first computed and the most dangerous intruder in each sector
isselected.Then,anewvelocityisgeneratedviathefollowingfuzzyinferenceprocess:(1)decideall
possiblepremises,thatis,allcombinationsofthefuzzysetsoftheselectedintruders’collisiontime;(2)
foreachpremise,activatethecorrespondingruleandreturnafuzzysetbasedon(7);(3)computethe
finalfuzzysetbasedon(8);and(4)computethespeedratioα∗ andtheorientationchange(cid:7)θ∗ based
on(9).Hence,thenewvelocitycanbedescribedas
vg=[α∗v cos(θ−(cid:7)θ∗),α∗v sin(θ−(cid:7)θ∗)]T (10)
i c c
Using the above three steps, a candidate velocity can be generated for each robot for collision
avoidance.
6.4. Velocityfine-tuning
Since fuzzy inference is a learning-based method depending on the quality of the sampling data, the
generated velocity may still result in a collision. Hence, fine-tuning is required to get a collision-free
velocity.
Incasethegeneratedvelocityforr isstillintheVOregionofsomeselectedintruders,thevelocity
i
shouldbeadjustedsuchthatitisoutoftheVOregion.Indeed,anyVO-basedmethodcanbeapplied,
suchasORCA,tocomputethecurrentvelocity.However,consideringthecomputationcost,aheuristic
fine-tuningmethodisproposed.AsshowninFig.9,vg isthevelocityobtainedbyfuzzyinference,BL
i
and BR are the boundaries of the VO region. There are two situations based on the robot’s current
position. The first one is shown in Fig. 9(a), where the current position p is out of the VO VO . In
i i|i1
thiscase,thesolutionistodecreasethegeneratedspeed(cid:7)vg(cid:7) totheboundaryofVO(i.e.,pointain
i 2
Fig.9(a))withoutchangingtheorientationofthevelocity.ThesecondoneisshowninFig.9(b),where
the current position p is in the VO VO . In this case, the main idea is to increase the speed and/or
i i|i1
changetheorientationofvg.However,sincethebuiltfuzzyrulesprefertodecreasethespeedtoavoid
i
collisions, the generated speed may be too small to guarantee collision avoidance by only changing
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 13 ---

680 WenbingTangetal.
Algorithm1:Updateofcollision-freevelocityforr attimeinstantt.
i
the orientation. Hence, for this situation, the speed will be increased to the boundary (i.e., point a in
Fig.9(b)).
7. Distributedalgorithmforcooperativecollisionavoidance
Thissectionsummarizesthedistributedcollisionavoidancealgorithmbasedonfuzzyrules,aswellas
theanalysisofthecomputationcomplexity.
Algorithm 1 describes the velocity computation at each time instant for a robot. In this algorithm,
Lines3−11classifytheintrudersintodifferentsectors.SincethereareN robotsinthesystem,inthe
r
worstcase,arobotneedstocheckallotherrobots.SothecomputationcomplexityisO(N).Lines12−22
r
select an intruder in each sector. Hence, the computation complexity is O(N) in the worst case. Line
r
23executefuzzyinference.Basedonthedescribedinferenceprocedure,arobotactivatesmultiplerules
withdifferentcombinationsoffuzzysetsoftheselectedintruderseachtime.Inageneralcase,suppose
eachrobotpartitionsitscollisionregionintoqsectors,andeachintruderisassignedwithf fuzzysets.
Hence, each time a robot activates at most fq rules, and the computation complexity is O(fq). Once
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 14 ---

Robotica 681
Algorithm2:Distributedcollisionavoidanceforacooperativemulti-robotsystem.
completing fuzzy inference, therobot computes the newvelocity and updates itscurrentvelocity, the
computationcomplexityisO(1).Hence,thetotalcomplexityofAlgorithm1isO(N +fq).
r
Algorithm2givesthedistributedcollisionavoidancealgorithmforacooperativemultirobotsystem.
Sector partition and fuzzy rule generation are done offline before a robot starts to perform collision
avoidanceandmoveforward.Thenumberoffuzzyrulesdependsononlythenumberofgeneratedfuzzy
setsandthenumberofdividedsectorsforeachrobot.Withqdividedsectorsandf generatedfuzzysets,
themaximalnumberoffuzzyrulesis(f +1)q−1.Eachrobotinasystemiscontrolledwiththesame
rule base. Lines 6−10 describe the motion control for each robot. Each robot executes this part in a
distributedwaybycommunicatingwithitsneighborstoretrievetheircurrentstates.Afterinitialization,
at each time instant t , a robot updates its velocity based on Algorithm 1 (Line 8), then moves with
k
thenewvelocityinthenexttimeduration[t ,t )(Line9).Whentimeelapses,therobotupdatesthe
k k+1
current time instant and check whether it arrives at its target position. If there are no intruders in its
sensingrange,therobotwillupdateitsspeedtovrefandmovedirectlytothetargetposition(Line12).
i
Note that since the rule generation depends on only the number of partitioned sectors of a robot’s
collisionregion,Fuzzy-VOissuitableforsystemswithdifferentnumbersofrobots,andevenforasystem
changingthenumberofrobotsinreal-time,suchasdeletingexistingrobotsoraddingnewrobotsduring
theevolutionofthesystem.However,Fuzzy-VOsuffersfromthechallengeofSim-to-Realtransfer.The
fundamentalprocedureintheproposedmethodistogenerateafuzzyrulebase,whichhighlydepends
onthesamplingdata.Usually,thetrainingdataarecollectedfromsimulationsinceitiscost-andtime-
consumingtocollectdatafromreal-worldexecution.DuetotheSim-to-Realgap,apracticalstrategyis
tofine-tunetherulebaseforreal-worldapplications.
8. Simulationresults
Inthissection,todemonstratetheefficiencyofFuzzy-VO,asetofsimulationswithdifferentnumbers
ofUAVsareconducted,equippedwithgenericodometrysensors,onRotorS,whichisamicroairvehi-
cle gazebo simulator with different multirotor models such as the AscTec Hummingbird, the AscTec
Pelican,ortheAscTecFirefly[42].Figure10showsasnapshotofasimulationwith6AscTecFirefly
UAVs. In simulations, all robots are flying to their target positions from initial positions at the same
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 15 ---

682 WenbingTangetal.
Figure10. ThesimulationenvironmentinGazebo.
height. Each robot communicates with others through topic subscription and publication in ROS. In
this way, the robot can subscribe to the states of others and publish its own state asynchronously in
real-time.Eventhoughassumethattherearenootherdynamicobstaclesintheenvironment,eachrobot
regardsothersasdynamicobstacles.Hence,thesimulationresultsinallexperimentscanalsoextendto
environmentswithvariousdynamicobstacles.
AllsimulationexperimentsarecarriedoutonaDellPrecisionTower5810desktoprunningUbuntu
16.04.6LTSandROSKinetic,andequippedwithIntel(R)Xeon(R)CPUE5-1650v4@3.60GHzand
32.0GBRAM.Inexperiments,thesaferadiusofeachrobotisρ=0.55unit;t andt inFig.6(a)are
1 2
equalto1.2and8s,respectively;theα inFig.6(b)is0.8;theinitialspeedofeachrobotis2unit/s;the
0
sensingrange L=8units;andthetimestep(cid:7)=0.01s,which satisfiesthedefault publishingrateof
topicsinRotorS.
8.1. Multiplerobotsmovinginthesameenvironment
ToevaluatetheeffectivenessofFuzzy-VO,thefirstsimulationexperimentisconductedwith4AscTec
Firefly UAVs, denoted as r − r . Their initial positions are (0,20), (0,0), (20,0), and (20,20),
1 4
respectively.Theyneedtomoveto(20,0),(20,20),(0,20),and(0,0),respectively.
Figure11showssomesnapshotsofthemotionofthefourUAVsatdifferenttimeinstants.Asshown
inFig.11(a),fromthestarttimetothetimeinstantk=342(i.e.,t=3.42s),eachUAVdetectsthatthere
arenointruderswithinitssensingrange,sotheymovedirectlytotheirtargets.Whenthetimeinstantis
k=428,r arrivesat(14.135,5.923)withaspeedofv =2units/sandanorientationθ =135.121◦.At
3 3 1
thistime,itdetectsthattherearetwointrudersinitssensingrange,namely,r andr ,whosepositions
2 4
are (6.218,6.331) and (13.808,13.748), respectively. With intruder selection, r finds that r is in its
3 2
RR,whiler isinitsLR.Thus,r activatesrules15−18togeneratethemotioncommand.Themotion
4 3
command generated by Fuzzy-VO for r at this time is: α=0.285, (cid:7)θ=43.423◦. Hence, as shown
3
in Fig. 11(b) and (c), r changes its motion to right from k=428 to k=449. Similarly, during the
3
motionfromk=449tok=474,r ,r ,andr detectintrudersintheirsensingrangessequentially,so
2 1 4
they turn a little bit to right to avoid collisions, as shown in Fig. 11(d). At k=474 (t=4.74s), r is
4
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 16 ---

Robotica 683
Figure11. DetailedsimulationprocessofcollisionavoidanceforfourUAVs.
at (12.787,12.756) and detects r , r , and r , whose positions are (7.141,12.635), (7.201,7.049), and
1 2 3
(13.546,6.924),respectively,areinitssensingrange,sotherearethreeintrudersneedtoaddressforr .
4
Clearly,r isinRR,r isinFR,andr isinLR.Sincethereisoneintruderineachsector,rules19−26
1 2 3
areactivated.Thegeneratedcommandis:α=0.267,(cid:7)θ=82.5◦.Hence,r shouldturngreatlytoright
4
andslowdownitsspeed,asshownbythepathsinFig.11(e).Att=5.87s,r detectsr isinitsFR,so
2 3
rules3and4areactivatedtogeneratecommand((cid:7)θ=23.279◦)forr ,whichbringsr toturnright.
2 2
Aftert=5.87s,r doesnotdetectanyintruderinitssensingrange,sor onlyneedstomovetowards
4 4
thetargetposition.SuchmotionofthemcanbefoundinFig.11(f).Whenk=667,thatis,t=6.67s,
theorientationofthefourUAVsare−41.618◦,48.85◦,140.068◦,and−129.561◦,respectively.Forr
3
andr ,theinitialorientationis135◦ and−135◦,respectively.BasedonthemotionshowninFig.11(f)
4
and(g),fromt=6.67stot=7.50s,eachrobotturnsalittlebittorightfirsttoavoidcollisionswith
otherrobots.Whentherearenointrudesinitssensingrange,eachrobotturnstolefttomovedirectlyto
itstarget.
As time elapses, the system reaches Fig. 11(g) at t=7.50 s. From now on, even through there are
othersUAVswithintheirsensingranges,thefourUAVsdetectthattherearenocollisionsinthefuture,
andallpotentialcollisionsareresolved.Hence,eachUAVmovesdirectlytoitstargetagain.Asshownin
Fig.11(h),allrobotsreachtheirtargetsatt=14.74s.Fromtheirtraversedtrajectories,anobservation
isthatinordertopasstheintersectionwithoutanycollision,eachrobotmovestoitsrightsidetoavoid
collisions.Fromtheirtraversedtrajectories,itiscanbeseenthatinordertopasstheintersectionwithout
anycollision,eachrobotmovestoitsrightsidetoavoidcollisions.Asinthegroundtrafficsystem,such
motionoftheserobotsissimilartothemotionwithinaroundabout[16].
Furthermore,moresimulationsarealsoconductedwith3,4,6,and8UAVs,respectively.Allvideos
ofsimulationscanbefoundathttps://fuzzyvo.github.io/.ItcanbeseenthatFuzzy-VOcanserveasan
effective and universal collision avoidance method, for multirobot systems with different numbers of
robots,byleveragingtheadvantagesoffuzzyrulesandVOs.
8.2. ComparisonofFuzzy-VOwithotherfuzzyrule-basedmethods
Sinceourmethodimprovesfuzzyrule-basedmethods,whichcandealwithuncertaintiesandlinguis-
tic variables, we compare Fuzzy-VO with the pure fuzzy rule method on the number of rules and the
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 17 ---

684 WenbingTangetal.
(a) (b)
Figure12. ThenumberofrulesandthegeneratedpathsofFuzzy-VOandpurefuzzyrulemethod.The
dottedlinesindicatethepathsofthedynamicobstacles.
smoothness of the generated paths. In addition, more experiments are also conducted to compare the
performanceoftheproposedintruderselectionmethodwithotherselectionmethods.
8.2.1.Comparisonwithpurefuzzyrulemethod
Let us first consider the number of rules. For the pure fuzzy control method, at any time instant, a
robotevaluatestherisksrelatedtoallotherrobotsandthendeterminesitsmotion.Hence,therulesare
generated directly based on the number of robots in a system. Indeed, in this method, the number of
rulesincreasesexponentiallywiththenumberofrobotsinthesystem.Supposetheriskrelatedtoeach
robotisdescribedbytwofuzzysets:dangerousandsafe,andtheorientationisdescribedbythreefuzzy
sets:left,front,andright.IfthereareM robotsinasystem,thepossiblerulesare6M−1.Forexample,
asshowninFig.12(a),whentherearethree,four,andfiverobots,respectively,therequirednumbers
ofrulesis36,261,and1296,respectively.However,inFuzzy-VO,byintruderselection,thenumberof
rulesisfixedanddoesnotchangewhenthenumberofrobotsislargerthan3.AsshowninFig.12(a),
whentherearetworobotsinthesystem,thereisoneobstacletobeprocessed,soonlysixrulesrequired
forbothFuzzy-VOandpurefuzzyrulecontrol.Thereare18ruleswhenthesystemcontainsthreerobots.
WhenM≥4,thenumberofrulesinFuzzy-VOisalways26(withoutconsideringtheemergentsituation
described before), which is independent of the number of robots. The result indicates that Fuzzy-VO
decreasesthenumberoffuzzyrulessignificantlybyselectingaproperandfixednumberofintruders.
Let us further compare the paths generated by the two methods. Consider the situation that there
are three dynamic obstacles (r −r ) and a Test Robot. The Test Robot is placed to (20,0) with an
1 3
initial speed is 2 units/s, and its target position is (20,40). The three obstacles’ initial positions are
(15,40), (18,40), and (25,40), respectively; their missions are that r −r need to move directly to
1 3
(25,0),(21,0),and(15,0),respectively,withafixedspeed2units/s.Figure12(b)showsthepathsthatthe
robotstraversed.ThesolidlinesrepresentthepathsoftheTestRobot.Fromthepaths,anobservationis
thatthepathgeneratedbyFuzzy-VOissmoother.Indeed,inpurefuzzyrulemethod,duetotheexcessive
consideringofthelargernumberoffuzzyrules,someunnecessaryrulesareactivatedtogeneratemotion
sometimes,whichmakethegeneratedpathoscillating.Theresultvalidatesthatthepathofarobotmay
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 18 ---

Robotica 685
be oscillating when there are a large number of rules. Since it generates a proper number of rules,
Fuzzy-VOcanproducesmootherpaths.
8.2.2.Performancecomparisonofdifferentselectionmethods
Thecommonmetricstoassesscollisionrisksinadynamicenvironmentinclude:distancetocollision
(DTC),timetocollision(TTC),andtimetoreact(TTR)[43].Asmentionedbefore,thispaperfocuses
on TTC, that is, potential collision time in Fuzzy-VO, to select the most dangerous intruder in each
sector. In some scenarios, a robot can only determine the nearest obstacle considered in each region
usingitsonboardsensorinformationdirectly,suchasthatofultrasonicsensorsandlasersensors.Thus,
thecollisionriskinevaluatedbasedonDTC,suchas[21].ForadeeperexplorationofFuzzy-VO,the
performancecomparisonbetweenthetwomethodsisstudied.Thesystemsareinitializedwith3,4,5,
6,8,and10robots,respectively,usingMonteCarlosimulation.Foreachsystem,10,000experiments
areperformed.Ineachexperiment,theinitialandtargetpositionsforeachrobotarerandomlyassigned.
Then,Fuzzy-VOandtheDTC-basedmethod(theconfigurationsoffuzzysets,membershipfunctions,
andfuzzyrulesarethesameas[21])arerunindependently.Thecomparisonmetricsareasfollows.
1. Successrate.Itisdefinedastheratioofthenumberofexperimentsinwhicheachrobotarrives
atitstargetpositionsuccessfullywhilemaintainingasafedistancewithotherrobotsatanytime
instant.
2. Minimumseparation.Itisdefinedastheminimumdistancebetweenanytworobotsduringtheir
motionineachcollision-freeexperiments.
3. Trajectory length. It is defined as the path length of a robot. Due to different initial and target
positions, the trajectory length is normalized by the distance between each pair of initial and
targetpositions.Hence,trajectorylengthcanbecomputedasfollows.
(cid:4)
(cid:7)p(k+1)−p(k)(cid:7)
L (i)= k i i 2 (11)
tr (cid:7)po−pf(cid:7)
i i 2
4. Motiontime.Itisdefinedasthediscretestepsperformedfromtheinitialpositiontothetarget
positionofarobot.
Foreachsystem,themeanofeachmetricandthecomparisonresultsareshowninFig.13.Asshown
inFig.13(a),withtheincreaseofrobots,DTC-basedselectioncannotbeappliedanymoresincemost
oftheexperimentsareincollisions,whileFuzzy-VOisstillwithalowcollisionrate.Inaddition,from
theaveragestatisticsaspect,comparedtotheDTC-basedselection,theaveragesuccessrateofFuzzy-
VOs can be increased by 306.5%. Note that the reasons for the happening of collisions in Fuzzy-VO
are that: (1) since the initial and target positions are generated randomly, they may be in collisions
when they are spawned; (2) currently, there are only two fuzzy sets associated to the collision time
in Fuzzy-VO, so a robot in some situations may not be able to respond to collisions timely; it can be
addressedbyrefiningthecollisiontimewithmorefuzzysets;however,themorefuzzysets,themore
fuzzyrules;furtherstudyonhowtobalancethenumberoffuzzysetsandmotionperformanceisone
offuturedirections.Figure13(b)showsthatFuzzy-VOcandealwithcollisionsmorepreciselysinceit
generates lowerminimumseparationinthecontextofcollisionavoidance. Figure 13(c)and(d)show
that Fuzzy-VO can generate lower trajectory length and shorter motion time to complete the motion
tasks.
Additionally, the variances of minimum separation, trajectory length, and steps are shown as in
TableII.Itcanbefoundthatinallexperiments,Fuzzy-VOcangenerateasmallervariancethanfuzzy
ruleswithDTC.Hence,itcanbeconcludedthattheproposedintruderselectionmethodcanassessthe
collisionriskaccuratelyandguaranteeahighersuccessrate.
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 19 ---

686 WenbingTangetal.
(a) (b)
(c) (d)
Figure13. ComparisonofmotionperformancebetweenFuzzy-VOandDTC-basedmethod.
8.3. ComparisonofFuzzy-VOwithORCA
At last, the comparison between Fuzzy-VO and ORCA [14] is carried out. It aims to investigate the
real-timedecisionefficiencyofFuzzy-VO.Theexperimentsonsixsystemsareconductedwithdiffer-
ent robots, that is, 3, 4, 5, 6, 8, and 10 robots, respectively. For each system, running Fuzzy-VO and
ORCA 10,000 times, respectively. Figure 14(a) shows different methods’ average success rates under
the control of the two methods. It can be observed that even though the success rate decreases as the
number of robots vary from 3 to 10, Fuzzy-VO can remain a high success rate. Note that in Fuzzy-
VO,itisatrade-offbetweenthenumberofselectedintrudersandthecollisionavoidanceperformance.
If the number of selected intruders is too large, the computation cost will increase, and the smooth-
nessofgeneratedtrajectorywilldecrease.Iftheselectedintrudersaretoofew,thegeneratedvelocities
may cause collisions. Hence, sometimes, users need to determine an appropriate number of collision
regions.
Tofurthercomparethereal-timeperformance,theaverageone-stepdecisiontime(averagetimecon-
sumedforgeneratingonecandidatevelocity)ofthetwomethodsiscalculated.Theresultsareshownin
Fig.14(b).Fromtheresults,itcanbefoundthatFuzzy-VOhasashorteraveragedecisiontimeateach
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 20 ---

Robotica 687
TableII. Comparisonofthevariancesofminimumseparation,trajectorylength,andsteps.
#robots Method Minimumseparation Trajectorylength #steps
3 Fuzzy-VO 126.7 1.472×103 5.146×101
FuzzyruleswithDTC 134.9 5.351×105 1.619×104
4 Fuzzy-VO 44.64 2.414×103 5.171×101
FuzzyruleswithDTC 53.99 4.692×106 2.382×104
5 Fuzzy-VO 6.967 2.293×103 2.181×101
FuzzyruleswithDTC 44.36 1.934×106 6.077×103
6 Fuzzy-VO 12.97 3.318×103 6.088×102
FuzzyruleswithDTC 25.05 5.936×106 1.316×104
8 Fuzzy-VO 4.377 4.546×103 2.565×101
FuzzyruleswithDTC 10.71 9.890×106 1.256×104
10 Fuzzy-VO 2.174 6.307×103 8.043×101
FuzzyruleswithDTC 7.843 1.216×106 9.748×103
(a) (b)
Figure14. ComparisonofavoidanceperformancebetweenFuzzy-VOandORCA.
timestep.Withtheincreaseofthenumberofrobots,Fuzzy-VOcanfiltermorenonurgentrobotsandhas
alessaverageone-stepdecisiontimeforeachrobot.ButthedecisiontimeofORCAmethodincreases
withthenumberofrobotsinasystem,asORCAmakesdecisionbyresolvinganoptimizationproblem
considering all other robots. The average one-step decision time of Fuzzy-VO is reduced by 740.9%,
comparedwiththeORCA.ThiscomparisonresultsalsofurthervalidatethatFuzzy-VOcanimprovethe
real-timedecisionefficiencywithoutlosingmuchsuccessrate.
9. Conclusion
Inthispaper,adistributedandhybridmethod,Fuzzy-VO,forcollisionavoidanceinmultirobotsystems
isproposed.TheproposedmethodleveragestheadvantagesoffuzzyrulesandVOssimultaneously.It
can deal with uncertain data and linguistic variables and also guarantee both safety and computation
efficiency.Itissuitableformultirobotsystemswithdifferentnumbersofrobots.Inthemethod,afixed
formandnumberoffuzzyrulesaregeneratedviacollisionregionpartitionandintruderselection.To
guaranteesafety,Fuzzy-VOappliesVO-basedfine-tuningtofurthercheckandadjustthevelocitygen-
eratedbyfuzzyinference.Experimentalresultswithdifferentscenariosshowa306.5%improvementin
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 21 ---

688 WenbingTangetal.
successrateanda740.9%reductionindecisiontime,demonstratingtheeffectivenessoftheproposed
method.
Future work includes implementing Fuzzy-VO on real robot platforms to further investigate the
performance of the proposed method. Another interesting topic is to study the combinations of
other model-driven methods and data-driven methods and compare their performance and scopes of
application.
Supplementary material. To view supplementary material for this article, please visit https://doi.org/10.1017/
S0263574722001515.
AuthorContributions. Allauthorsconceivedanddesignedthestudy.WenbingTangandYuanZhouimplementedtheresearch
andwrotethemanuscript.ZuohuaDingandJingLiudevelopedthefuzzy-logiccontroller.TianweiZhangandYangLiureviewed
andeditedthemanuscript.
FinancialSupport. ThisworkwassupportedbytheNaturalScienceFoundationofChinaunderGrantNos.61751210,61210004
and61170015,AcademicResearchFundTier2byMinistryofEducationinSingaporeunderGrantNo.MOE-T2EP20120-0004.
ConflictsofInterest. Theauthorsdeclarethatthereisnoconflictofinterests.
EthicalApproval. None.
References
[1] L.Jin,Y.Qi,X.Luo,S.LiandM.Shang,“Distributedcompetitionofmulti-robotcoordinationundervariableandswitching
topologies,”IEEETrans.Autom.Sci.Eng.19(4),3575–3586(2022).doi:10.1109/TASE.2021.3126385.
[2] Z.Zhou,J.LiuandJ.Yu,“Asurveyofunderwatermulti-robotsystems,”IEEE-CAAJ.Autom.Sin.9(1),1–18(2022).
[3] N.Nfaileh,K.Alipour,B.TarvirdizadehandA.Hadi,“Formationcontrolofmultiplewheeledmobilerobotsbasedon
modelpredictivecontrol,”Robotica40(9),1–36(2022).
[4] L.ZhouandP.Tokekar,“Activetargettrackingwithself-triggeredcommunicationsinmulti-robotteams,”IEEETrans.
Autom.Sci.Eng.16(3),1085–1096(2019).
[5] K.Brown,O.Peltzer,M.A.Sehr,M.SchwagerandM.J.Kochenderfer,“OptimalSequentialTaskAssignmentandPath
FindingforMulti-AgentRoboticAssemblyPlanning,”In:IEEEInternationalConferenceonRoboticsandAutomation,
Pairs,France(2020)pp.441–447.
[6] S.H.Jazi,M.Keshmiri,F.Sheikholeslam,M.G.ShahrezaandM.Keshmiri,“Adaptivemanipulationandslippagecontrol
ofanobjectinamulti-robotcooperativesystem,”Robotica32(5),783–802(2014).
[7] P.YuandD.V.Dimarogonas,“DistributedmotioncoordinationformultirobotsystemsunderLTLspecifications,”IEEE
Trans.Robot.38(2),1047–1062(2022).
[8] Y.Kantaros,M.Malencia,V.KumarandG.J.Pappas,“ReactiveTemporalLogicPlanningforMultipleRobotsinUnknown
Environments,”In:IEEEInternationalConferenceonRoboticsandAutomation,Pairs,France(2020)pp.11479–11485.
[9] Y.Zhou,H.Hu,Y.LiuandZ.Ding,“Collisionanddeadlockavoidanceinmultirobotsystems:Adistributedapproach,”
IEEETrans.Syst.ManCybern.Syst.47(7),1712–1726(2017).
[10] Y.Zhou,H.Hu,Y.Liu,S.-W.LinandZ.Ding,“Adistributedapproachtorobustcontrolofmulti-robotsystems,”Automatica
98(6),1–13(2018).
[11] Y.Zhou,H.Hu,Y.Liu,S.-W.LinandZ.Ding,“Adistributedmethodtoavoidhigher-orderdeadlocksinmulti-robot
systems,”Automatica112,108706:1–108706:13(2020).
[12] H.G.TannerandA.Boddu,“Multiagentnavigationfunctionsrevisited,”IEEETrans.Robot.28(6),1346–1359(2012).
[13] J.VandenBerg,M.LinandD.Manocha,“ReciprocalVelocityObstaclesforReal-TimeMulti-AgentNavigation,”In:IEEE
InternationalConferenceonRoboticsandAutomation,Pasadena,California,USA(2008)pp.1928–1935.
[14] J.vandenBerg,S.J.Guy,M.LinandD.Manocha,“Reciprocaln-bodycollisionavoidance,”Robot.Res.70,3–19(2011).
[15] B.Lindqvist,S.S.Mansouri,A.-A.Agha-MohammadiandG.Nikolakopoulos,“NonlinearMPCforcollisionavoidance
andcontrolofUAVswithdynamicobstacles,”IEEERobot.Autom.Lett.5(4),6001–6008(2020).
[16] Y.Zhou,H.Hu,Y.Liu,S.-W.LinandZ.Ding,“Areal-timeandfullydistributedapproachtomotionplanningformultirobot
systems,”IEEETrans.Syst.ManCybern.Syst.49(12),2636–2650(2019).
[17] B.Tang,K.Xiang,M.PangandZ.Zhanxia,“Multi-robotpathplanningusinganimprovedself-adaptiveparticleswarm
optimization,”Int.J.Adv.Robot.Syst.17(5),1–19(2020).
[18] N.ThumigerandM.Deghat,“Amulti-agentdeepreinforcementlearningapproachforpracticaldecentralizedUAVcollision
avoidance,”IEEEControlSyst.Lett.6,22174–22179(2022).
[19] R.Han,S.Chen,S.Wang,Z.Zhang,R.Gao,Q.HaoandJ.Pan,“Reinforcementlearneddistributedmulti-robotnavigation
withreciprocalvelocityobstacleshapedrewards,”IEEERobot.Autom.Lett.7(3),5896–5903(2022).
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press


--- Page 22 ---

Robotica 689
[20] J.H.Lilly,“Evolutionofanegative-rulefuzzyobstacleavoidancecontrollerforanautonomousvehicle,”IEEETrans.Fuzzy
Syst.15(4),718–728(2007).
[21] Z.M.Wen,S.D.ZhouandM.Wang,“FuzzycontrolfortheobstacleavoidanceofaquadrotorUAV,”Appl.Mech.Mater.
775,307–313(2015).
[22] T.Edward,“MobileRobotAutonomyviaHierarchicalFuzzyBehaviorControl,”In:InternationalSymposiumonRobotics
andManufacturing,Montpellier,France(1996)pp.837–842.
[23] C.Wang,A.V.SavkinandM.Garratt,“Astrategyforsafe3Dnavigationofnon-holonomicrobotsamongmovingobstacles,”
Robotica36(2),275–297(2018).
[24] C.KownackiandL.Ambroziak,“Anewmultidimensionalrepulsivepotentialfieldtoavoidobstaclesbynonholonomic
UAVsindynamicenvironments,”Sensors21(22),7495(2021).
[25] D.F.Llorca,V.Milanés,I.P.Alonso,M.Gavilán,I.G.Daza,J.PérezandM.Á.Sotelo,“Autonomouspedestriancollision
avoidanceusingafuzzysteeringcontroller,”IEEETrans.Intell.Transp.Syst.12(2),390–401(2011).
[26] P.Vadakkepat,O.C.Miin,X.PengandT.H.Lee,“Fuzzybehavior-basedcontrolofmobilerobots,”IEEETrans.Fuzzy
Syst.12(4),559–565(2004).
[27] Y.-C.Chang,Y.Shi,A.Dostovalova,Z.Cao,J.Kim,D.GibbonsandC.-T.Lin,“Interpretablefuzzylogiccontrolfor
multirobotcoordinationinaclutteredenvironment,”IEEETrans.FuzzySyst.29(12),3676–3685(2021).
[28] P.FioriniandZ.Shiller,“MotionPlanninginDynamicEnvironmentsUsingtheRelativeVelocityParadigm,”In:IEEE
InternationalConferenceonRoboticsandAutomation,Atlanta,GA,USA(1993)pp.560–565.
[29] M. Kim and J.-H. Oh, “Study on optimal velocity selection using velocity obstacle (OVVO) in dynamic and crowded
environment,”Auton.Robot.40(8),3676–3685(2016).
[30] K.Cai,C.Wang,J.Cheng,C.W.DeSilvaandM.Q.H.Meng,“Mobilerobotpathplanningindynamicenvironments:A
survey,”arXivpreprintarXiv:2006.14195,(2020).
[31] Y. I. Jenie, E.-J. Kampen, C. C. de Visser, J. Ellerbroek and J. M. Hoekstra, “Selective velocity obstacle method for
deconflictingmaneuversappliedtounmannedaerialvehicles,”J.Guid.ControlDyn.38(6),1140–1146(2015).
[32] S.Wang,Z.Li,B.Wang,J.MaandJ.Yu,“Velocityobstacle-basedcollisionavoidanceandmotionplanningframeworkfor
connectedandautomatedvehicles,”Transp.Res.Rec.2676(5),748–766(2022).
[33] J. A. Douthwaite, S. Zhao and L. S. Mihaylova, “Velocity obstacle approaches for multi-agent collision avoidance,”
UnmannedSyst.7(1),55–64(2019).
[34] M.Shen,Y.Wang,Y.Jiang,H.Ji,B.WangandZ.Huang,“Anewpositioningmethodbasedonmultipleultrasonicsensors
forautonomousmobilerobot,”Sensors20(1),237–252(2019).
[35] G.Liu,M.Yao,L.ZhangandC.Zhang,“FuzzyControllerforObstacleAvoidanceinElectricWheelchairwithUltrasonic
Sensors,”In:InternationalSymposiumonComputerScienceandSociety,KotaKinabalu,Malaysia(2011)pp.71–74.
[36] Z.Ding,Y.ZhouandM.Zhou,“Modelingself-adaptivesoftwaresystemsbyfuzzyrulesandPetrinets,”IEEETrans.Fuzzy
Syst.26(2),967–984(2018).
[37] V. Kreinovich, O. Kosheleva and S. N. Shahbazova, “Why triangular and trapezoid membership functions: A simple
explanation,”RecentDev.FuzzyLogicFuzzySets391,25–31(2020).
[38] S.PingandZ.Yu,“Trackingcontrolforacushionrobotbasedonfuzzypathplanningwithsafeangularvelocity,”IEEE-CAA
J.Autom.Sin.4(4),610–619(2017).
[39] Z.Ding,Y.Zhou,G.PuandM.Zhou,“Onlinefailurepredictionforrailwaytransportationsystemsbasedonfuzzyrules
anddataanalysis,”IEEETrans.Reliab.67(3),1143–1158(2018).
[40] M.Figueiredo,F.Gomide,A.RochaandR.Yager,“ComparisonofYager’slevelsetmethodforfuzzylogiccontrolwith
Mamdani’sandLarsen’smethods,”IEEETrans.FuzzySyst.1(2),156–159(1993).
[41] T.JiangandY.Li,“Generalizeddefuzzificationstrategiesandtheirparameterlearningprocedures,”IEEETrans.FuzzySyst.
4(1),64–71(1996).
[42] F.Furrer,M.Burri,M.AchtelikandR.Siegwart,“Rotors—AmodulargazeboMAVsimulatorframework,”RobotOper.
Syst.625,595–625(2016).
[43] C.Katrakazas,M.Quddus,W.-H.ChenandL.Deka,“Real-timemotionplanningmethodsforautonomouson-roaddriving:
State-of-the-artandfutureresearchdirections,”Transp.Res.PartCEmerg.60,416–442(2015).
Citethisarticle:W.Tang,Y.Zhou,T.Zhang,Y.Liu,J.LiuandZ.Ding(2023).“Cooperativecollisionavoidanceinmultirobot
systemsusingfuzzyrulesandvelocityobstacles”,Robotica41,668–689.https://doi.org/10.1017/S0263574722001515
https://doi.org/10.1017/S0263574722001515 Published online by Cambridge University Press