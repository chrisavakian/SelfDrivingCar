#define MIC_INPUT                   A2
#define RXLED                       17
#define TXLED                       30

#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.5
#define BASIS_DIM                   3

#define EUCLIDEAN_THRESHOLD         0.035
#define LOUDNESS_THRESHOLD          80

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[80] = {-0.02675344410983409, -0.031006520827739914, -0.04528198045819232, -0.04732615632655153, -0.031101634972421778, -0.09516323618748809, -0.11643588811979892, -0.12570647499571155, -0.16306153637395795, -0.2175253572781774, -0.24088287897851265, -0.28232664007593, -0.27113525710450503, -0.28517005810028534, -0.26794177091744226, -0.21672875092103439, -0.10712966112271304, -0.001621966124670296, 0.06104418391109945, 0.0749963605421943, 0.07225983257581628, 0.034259793025656474, -0.014368671076258216, -0.017380861775253803, -0.014230427720277558, -0.027917769889370305, -0.03592707400977113, -0.01574353045206584, 0.02928005091859393, 0.07233225063042971, 0.10352606628966625, 0.14677127749828853, 0.17042287649951565, 0.21248653491592862, 0.23849853269106303, 0.2314053439762174, 0.22381521292744078, 0.214635041136048, 0.19315887394785536, 0.1577702442587913, 0.13192399142608388, 0.11385607494031143, 0.09409611487734865, 0.07472403907997832, 0.060150035229922454, 0.04474554954457611, 0.031521685320915135, 0.022957088691759647, 0.019268369991417972, 0.018300510812273275, 0.017478140132112654, 0.015375049508842718, 0.009151346568998119, 0.010424643780311374, 0.007042121125477581, 0.010804218412623327, 0.0054011897179495035, 0.0084471594248773, 0.005909036570179697, 0.006075316101499333, 0.003566474721537116, -0.0009377795265655642, -2.234547541053251e-05, -0.00691030117544218, -0.007982288444761593, -0.006777341735715089, -0.0066525128568078115, -0.017153809560069396, -0.011917100034672577, -0.012972346270875193, -0.014563541472616941, -0.012657599027723362, -0.01911357984722699, -0.016564696025827726, -0.019056376552335816, -0.01948424541012476, -0.019708044578050574, -0.015209709401953321, -0.022585450803027835, -0.01974401560643024};
float pca_vec2[80] = {0.037811123445457, 0.01918644387044599, -0.03614781693733271, -0.08502687026117994, -0.08074105222349073, -0.07067544005860144, -0.08594037947971116, -0.07874743894994363, -0.04903630596721324, -0.02527130988893897, 0.013986300091571646, 0.017899573048231546, 0.06022777659193727, 0.05031467684861409, 0.04563787599624964, -0.05428318740015989, -0.1934774785202391, -0.3170307691892385, -0.3412549269950881, -0.3668824072139146, -0.3745390134345945, -0.3570498660817494, -0.2807203614320374, -0.1420435442226605, -0.004524539669801141, 0.11517654885293524, 0.16328210684749642, 0.17123208996508485, 0.1340634892532437, 0.08948834131081305, 0.0584120753470757, 0.03196188058403064, 0.017323598836318357, -0.001262804615656982, -0.005598079971953534, 0.006759104926875407, 0.013552730251056922, 0.012354176753871822, 0.006599879652596554, 0.027673911753718358, 0.03388808981142496, 0.051675820874730126, 0.05412370015956865, 0.0602743902588595, 0.054956974068271144, 0.05977358924820401, 0.05495419675868959, 0.061208225261249555, 0.05885776688311411, 0.06163944716621642, 0.06302339848180685, 0.059206205453252794, 0.05321953205084253, 0.05565037104885383, 0.061801062137598645, 0.0540773329096429, 0.05531935486165816, 0.06340819563972107, 0.05332399960306194, 0.051521806033562975, 0.05084308491521558, 0.055288204220063995, 0.05553295235399677, 0.0493971541040969, 0.040274857420054704, 0.04330304215082545, 0.04566020369606946, 0.044549175272760776, 0.03746200475023003, 0.03403028238821719, 0.02660858620672232, 0.028866809848940717, 0.03151754572708224, 0.028390523786169682, 0.027170249453042764, 0.03187731468717104, 0.02509060175302675, 0.024669044639240925, 0.027799382957337124, 0.027075409245287493};
float pca_vec3[80] = {-0.013331014924083687, -0.01165155808634838, -0.008340728825015686, -0.035374441147078634, -0.09882043984090949, -0.08256250177438562, -0.10473816414964521, -0.11584014754132162, -0.06770243726422212, -0.034748229310538385, -0.001567902777523439, -0.020568093664558838, 0.0020085027952635144, -0.00258251819479516, -0.011693509040669977, -0.030471376551138407, -0.015689565376598727, 0.012706074840588118, 0.04326578627942828, 0.047995513593837774, 0.017256431565656226, 0.017401346343962575, -0.015050082288653482, -0.08734644414180008, -0.13115034796368424, -0.16666287491791598, -0.17813125469342922, -0.20597707653765512, -0.2582336980850788, -0.2801664837880874, -0.2873462798827045, -0.28989815543605413, -0.2740092406072855, -0.22266029557873238, -0.14514914550893468, -0.06671114104994667, 0.017539768223318042, 0.04909118261696432, 0.09405877502474412, 0.12078464258552123, 0.10946075742577888, 0.13068180658317394, 0.13627188980069285, 0.13355624378345612, 0.14191344618405505, 0.13939296997228987, 0.14102212223188137, 0.13971997888596452, 0.13853350289567443, 0.13645700208570627, 0.12639086032021432, 0.12334367964536767, 0.11845574102598164, 0.12502950081780548, 0.10956145941351494, 0.10759880910199286, 0.09550770409487544, 0.10038046532020314, 0.09680675296448421, 0.09002600901216971, 0.08619280022903734, 0.06952790941262171, 0.0593698690933324, 0.05857937395058016, 0.05474165916900572, 0.03945970264016017, 0.036118296236886484, 0.026973773200654343, 0.014454395012168752, 0.004302498310221973, -0.003853818570831529, -0.0013571633169160457, -0.011657564229724348, -0.0032315382498232367, -0.01395386984979515, -0.003338393617466458, 0.00021115781155381917, -0.0054275976828057374, -0.0020125344058326156, -0.003142531628798934};
float projected_mean_vec[3] = {-0.035364329093461294, -0.041277328273088054, -0.023047062846825887};
float centroid1[3] = {-0.05212525194598892, -0.03971890015189157, 0.0023931645811224693};
float centroid2[3] = {-0.03259960211059743, 0.04292979138031921, -0.02009292764847539};
float centroid3[3] = {0.0231697406943981, 0.012859016334889251, 0.04218617173654632};
float centroid4[3] = {0.061555113362188274, -0.01606990756331687, -0.024486408669193403};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};
volatile int re_pointer = 0;

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
int write_arr = 0;

// this function allows us to get the correct array to save data to
// we are using a concept known as "pointer juggling" to save memory
int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  pinMode(MIC_INPUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(1000);

  re_pointer = 0;
      
  cli();
 
  //set timer1 interrupt at 1Hz * SAMPLING INTERVAL / 1000
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();

  Serial.begin(38400);
  delay(1000);
}


void loop(void) {
  if (re_pointer%AVG_SIZE == 0 && re_pointer <= SIZE){
    write_arr = !write_arr;
    envelope(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);
    
    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if(envelope(out, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: the principal components are unit norm
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += result[i]*pca_vec1[i];
        proj2 += result[i]*pca_vec2[i];
        proj3 += result[i]*pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];

      // Classification
      // Use the function 'l2_norm3' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      for (int i = 0; i < 4; i++){
          float distance = l2_norm3(proj1,proj2,proj3,centroids[i]);
          if (distance < best_dist){
              best_dist = distance;
              best_index = i+1; 
          }
      }
      

      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      String ourWords[] = {"Alex","Cracker","California","ScoobyDoo"};
      if (best_dist < EUCLIDEAN_THRESHOLD) {
          Serial.println("Found a word! Word " + ourWords[best_index-1]);
      } else {
        Serial.println("Below EUCLIDEAN_THRESHOLD - " + String(best_dist));
      }
      

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
   } else {
     Serial.println("Below LOUDNESS_THRESHOLD.");
   }

    delay(2000);
    re_pointer = 0;
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(RXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, LOW);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, LOW);
  delay(100);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  delay(100);
  digitalWrite(TXLED, HIGH);
}

void envelope(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
  
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
  
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    Serial.println(maximum);
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres && block < SIZE_AFTER_FILTER);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}
/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/


ISR(TIMER1_COMPA_vect){//timer1 interrupt 8Khz toggles pin 13 (LED)
  if (re_pointer < SIZE) {
    digitalWrite(RXLED, LOW);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}
