
*/
 
int Filter_Value;
int Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
  Value = 300;
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Value = Filter_Value;          // ���һ����Ч������ֵ���ñ���Ϊȫ�ֱ���
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// �޷��˲������ֳƳ����ж��˲�����
#define FILTER_A 1
int Filter() {
  int NewValue;
  NewValue = Get_AD();
  if(((NewValue - Value) > FILTER_A) || ((Value - NewValue) > FILTER_A))
    return Value;
  else
    return NewValue;
}


/*2����λֵ�˲���*/
/*
A�����ƣ���λֵ�˲���
B��������
    ��������N�Σ�Nȡ����������N�β���ֵ����С���У�
    ȡ�м�ֵΪ������Чֵ��
C���ŵ㣺
    ����Ч�˷���żȻ��������Ĳ������ţ�
    ���¶ȡ�Һλ�ı仯�����ı�����������õ��˲�Ч����
D��ȱ�㣺
    ���������ٶȵȿ��ٱ仯�Ĳ������ˡ�
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// ��λֵ�˲���
#define FILTER_N 101
int Filter() {
  int filter_buf[FILTER_N];
  int i, j;
  int filter_temp;
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = Get_AD();
    delay(1);
  }
  // ����ֵ��С�������У�ð�ݷ���
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  return filter_buf[(FILTER_N - 1) / 2];
}

/*3������ƽ���˲���*/
/*
A�����ƣ�����ƽ���˲���
B��������
    ����ȡN������ֵ��������ƽ�����㣺
    Nֵ�ϴ�ʱ���ź�ƽ���Ƚϸߣ��������Ƚϵͣ�
    Nֵ��Сʱ���ź�ƽ���Ƚϵͣ��������Ƚϸߣ�
    Nֵ��ѡȡ��һ��������N=12��ѹ����N=4��
C���ŵ㣺
    �����ڶ�һ�����������ŵ��źŽ����˲���
    �����źŵ��ص�����һ��ƽ��ֵ���ź���ĳһ��ֵ��Χ�������²�����
D��ȱ�㣺
    ���ڲ����ٶȽ�����Ҫ�����ݼ����ٶȽϿ��ʵʱ���Ʋ����ã�
    �Ƚ��˷�RAM��
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// ����ƽ���˲���
#define FILTER_N 12
int Filter() {
  int i;
  int filter_sum = 0;
  for(i = 0; i < FILTER_N; i++) {
    filter_sum += Get_AD();
    delay(1);
  }
  return (int)(filter_sum / FILTER_N);
}

/*4������ƽ���˲������ֳƻ���ƽ���˲�����*/
/*
A�����ƣ�����ƽ���˲������ֳƻ���ƽ���˲�����
B��������
    ������ȡ�õ�N������ֵ����һ�����У����еĳ��ȹ̶�ΪN��
    ÿ�β�����һ�������ݷ����β�����ӵ�ԭ�����׵�һ�����ݣ��Ƚ��ȳ�ԭ�򣩣�
    �Ѷ����е�N�����ݽ�������ƽ�����㣬����µ��˲������
    Nֵ��ѡȡ��������N=12��ѹ����N=4��Һ�棬N=4-12���¶ȣ�N=1-4��
C���ŵ㣺
    �������Ը��������õ��������ã�ƽ���ȸߣ�
    �����ڸ�Ƶ�񵴵�ϵͳ��
D��ȱ�㣺
    �����ȵͣ���żȻ���ֵ������Ը��ŵ��������ýϲ
    �������������������������Ĳ���ֵƫ�
    ��������������űȽ����صĳ��ϣ�
    �Ƚ��˷�RAM��
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// ����ƽ���˲������ֳƻ���ƽ���˲�����
#define FILTER_N 12
int filter_buf[FILTER_N + 1];
int Filter() {
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N] = Get_AD();
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
    filter_sum += filter_buf[i];
  }
  return (int)(filter_sum / FILTER_N);
}

/*5.��λֵƽ���˲���(�ֳƷ��������ƽ���˲���)*/
/*
A�����ƣ���λֵƽ���˲������ֳƷ��������ƽ���˲�����
B��������
    ��һ�����ȥ�����ֵ����Сֵ��ȡƽ��ֵ��
    �൱�ڡ���λֵ�˲�����+������ƽ���˲�������
    ��������N�����ݣ�ȥ��һ�����ֵ��һ����Сֵ��
    Ȼ�����N-2�����ݵ�����ƽ��ֵ��
    Nֵ��ѡȡ��3-14��
C���ŵ㣺
    �ں��ˡ���λֵ�˲�����+������ƽ���˲����������˲������ŵ㡣
    ����żȻ���ֵ������Ը��ţ�����������������Ĳ���ֵƫ�
    �����ڸ��������õ��������á�
    ƽ���ȸߣ����ڸ�Ƶ�񵴵�ϵͳ��
D��ȱ�㣺
    �����ٶȽ�����������ƽ���˲���һ����
    �Ƚ��˷�RAM��
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// ��λֵƽ���˲������ֳƷ��������ƽ���˲��������㷨1��
#define FILTER_N 100
int Filter() {
  int i, j;
  int filter_temp, filter_sum = 0;
  int filter_buf[FILTER_N];
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = Get_AD();
    delay(1);
  }
  // ����ֵ��С�������У�ð�ݷ���
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf[i] > filter_buf[i + 1]) {
        filter_temp = filter_buf[i];
        filter_buf[i] = filter_buf[i + 1];
        filter_buf[i + 1] = filter_temp;
      }
    }
  }
  // ȥ�������С��ֵ����ƽ��
  for(i = 1; i < FILTER_N - 1; i++) filter_sum += filter_buf[i];
  return filter_sum / (FILTER_N - 2);
}
 
 
//  ��λֵƽ���˲������ֳƷ��������ƽ���˲��������㷨2��
/*
#define FILTER_N 100
int Filter() {
  int i;
  int filter_sum = 0;
  int filter_max, filter_min;
  int filter_buf[FILTER_N];
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = Get_AD();
    delay(1);
  }
  filter_max = filter_buf[0];
  filter_min = filter_buf[0];
  filter_sum = filter_buf[0];
  for(i = FILTER_N - 1; i > 0; i--) {
    if(filter_buf[i] > filter_max)
      filter_max=filter_buf[i];
    else if(filter_buf[i] < filter_min)
      filter_min=filter_buf[i];
    filter_sum = filter_sum + filter_buf[i];
    filter_buf[i] = filter_buf[i - 1];
  }
  i = FILTER_N - 2;
  filter_sum = filter_sum - filter_max - filter_min + i / 2; // +i/2 ��Ŀ����Ϊ����������
  filter_sum = filter_sum / i;
  return filter_sum;
}*/

/*6.�޷�ƽ���˲���*/
/*
A�����ƣ��޷�ƽ���˲���
B��������
    �൱�ڡ��޷��˲�����+������ƽ���˲�������
    ÿ�β��������������Ƚ����޷�������
    ��������н��е���ƽ���˲�������
C���ŵ㣺
    �ں��������˲������ŵ㣻
    ����żȻ���ֵ������Ը��ţ������������������������Ĳ���ֵƫ�
D��ȱ�㣺
    �Ƚ��˷�RAM��
E��������shenhaiyu 2013-11-01
*/
 
#define FILTER_N 12
int Filter_Value;
int filter_buf[FILTER_N];
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
  filter_buf[FILTER_N - 2] = 300;
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// �޷�ƽ���˲���
#define FILTER_A 1
int Filter() {
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N - 1] = Get_AD();
  if(((filter_buf[FILTER_N - 1] - filter_buf[FILTER_N - 2]) > FILTER_A) || ((filter_buf[FILTER_N - 2] - filter_buf[FILTER_N - 1]) > FILTER_A))
    filter_buf[FILTER_N - 1] = filter_buf[FILTER_N - 2];
  for(i = 0; i < FILTER_N - 1; i++) {
    filter_buf[i] = filter_buf[i + 1];
    filter_sum += filter_buf[i];
  }
  return (int)filter_sum / (FILTER_N - 1);
}

/*7.һ���ͺ��˲���*/
/*
A�����ƣ�һ���ͺ��˲���
B��������
    ȡa=0-1�������˲����=(1-a)*���β���ֵ+a*�ϴ��˲������
C���ŵ㣺
    �������Ը��ž������õ��������ã�
    �����ڲ���Ƶ�ʽϸߵĳ��ϡ�
D��ȱ�㣺
    ��λ�ͺ������ȵͣ�
    �ͺ�̶�ȡ����aֵ��С��
    ���������˲�Ƶ�ʸ��ڲ���Ƶ��1/2�ĸ����źš�
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
int Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
  Value = 300;
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// һ���ͺ��˲���
#define FILTER_A 0.01
int Filter() {
  int NewValue;
  NewValue = Get_AD();
  Value = (int)((float)NewValue * FILTER_A + (1.0 - FILTER_A) * (float)Value);
  return Value;
}

/*8.��Ȩ����ƽ���˲���*/
/*
A�����ƣ���Ȩ����ƽ���˲���
B��������
    �ǶԵ���ƽ���˲����ĸĽ�������ͬʱ�̵����ݼ��Բ�ͬ��Ȩ��
    ͨ���ǣ�Խ�ӽ���ʱ�̵����ݣ�Ȩȡ��Խ��
    �����²���ֵ��Ȩϵ��Խ����������Խ�ߣ����ź�ƽ����Խ�͡�
C���ŵ㣺
    �������нϴ��ͺ�ʱ�䳣���Ķ��󣬺Ͳ������ڽ϶̵�ϵͳ��
D��ȱ�㣺
    ���ڴ��ͺ�ʱ�䳣����С���������ڽϳ����仯�������źţ�
    ����Ѹ�ٷ�Ӧϵͳ��ǰ���ܸ��ŵ����س̶ȣ��˲�Ч���
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// ��Ȩ����ƽ���˲���
#define FILTER_N 12
int coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // ��Ȩϵ����
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // ��Ȩϵ����
int filter_buf[FILTER_N + 1];
int Filter() {
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N] = Get_AD();
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  return filter_sum;
}

/*9.�����˲���*/
/*
A�����ƣ������˲���
B��������
    ����һ���˲�����������ÿ�β���ֵ�뵱ǰ��Чֵ�Ƚϣ�
    �������ֵ=��ǰ��Чֵ������������㣻
    �������ֵ<>��ǰ��Чֵ���������+1�����жϼ������Ƿ�>=����N���������
    ���������������򽫱���ֵ�滻��ǰ��Чֵ�������������
C���ŵ㣺
    ���ڱ仯�����ı�������нϺõ��˲�Ч����
    �ɱ������ٽ�ֵ�����������ķ�����/����������ʾ������ֵ������
D��ȱ�㣺
    ���ڿ��ٱ仯�Ĳ������ˣ�
    ����ڼ������������һ�β�������ֵǡ���Ǹ���ֵ,��Ὣ����ֵ������Чֵ����ϵͳ��
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
int Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
  Value = 300;
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// �����˲���
#define FILTER_N 12
int i = 0;
int Filter() {
  int new_value;
  new_value = Get_AD();
  if(Value != new_value) {
    i++;
    if(i > FILTER_N) {
      i = 0;
      Value = new_value;
    }
  }
  else
    i = 0;
  return Value;
}

 /*10.�޷������˲���*/
/*
A�����ƣ��޷������˲���
B��������
    �൱�ڡ��޷��˲�����+�������˲�������
    ���޷�����������
C���ŵ㣺
    �̳��ˡ��޷����͡����������ŵ㣻
    �Ľ��ˡ������˲������е�ĳЩȱ�ݣ����⽫����ֵ����ϵͳ��
D��ȱ�㣺
    ���ڿ��ٱ仯�Ĳ������ˡ�
E��������shenhaiyu 2013-11-01
*/
 
int Filter_Value;
int Value;
 
void setup() {
  Serial.begin(9600);       // ��ʼ������ͨ��
  randomSeed(analogRead(0)); // �����������
  Value = 300;
}
 
void loop() {
  Filter_Value = Filter();       // ����˲������ֵ
  Serial.println(Filter_Value); // �������
  delay(50);
}
 
// �����������һ��300���ҵĵ�ǰֵ
int Get_AD() {
  return random(295, 305);
}
 
// �޷������˲���
#define FILTER_A 1
#define FILTER_N 5
int i = 0;
int Filter() {
  int NewValue;
  int new_value;
  NewValue = Get_AD();
  if(((NewValue - Value) > FILTER_A) || ((Value - NewValue) > FILTER_A))
    new_value = Value;
  else
    new_value = NewValue;
  if(Value != new_value) {
    i++;
    if(i > FILTER_N) {
      i = 0;
      Value = new_value;
    }
  }
  else
    i = 0;
  return Value;
}














































