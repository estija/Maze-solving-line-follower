#define mlf 23
#define mlb 22
#define mlp 11
#define mrf 26
#define mrb 27
#define mrp 10
#define ledr 2
#define ledg 4
#define ledb 5
#define ledc 3

int mlpwm=0, mrpwm=0, mls=180, mrs=180, mlt=175, mrt=175; // mlt=65; mrt = 75;
int reading[7];
int dread[7];
int err=0, perr=0, derr=0, corr=0, kp=25, kd=15, sen_thr=380, lt_ov=100,rt_ov=100;
long long int t=0,tst=0,t_back=0,tl=0, distance=0;
int line_ov=160;
int end_dry=0, wsum=0, sum=0;
int D=0;
int i=1,j=1,l=0;
int dir[50],dist[50],x[50],y[50];
bool left,right,dead_end,all_white,node,turn,st;

void setup(){
 Serial.begin(9600);
  pinMode(mlf, OUTPUT);
  pinMode(mlb, OUTPUT);
  pinMode(mlp, OUTPUT);
  pinMode(mrp, OUTPUT);
  pinMode(mrf, OUTPUT);
  pinMode(mrb, OUTPUT);
  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledb, OUTPUT);
  pinMode(ledc, OUTPUT);
  digitalWrite(ledc,HIGH);
  digitalWrite(ledr,HIGH);
  digitalWrite(ledg,HIGH);
  digitalWrite(ledb,HIGH);
}
    
    void sensor_read(){
    reading[0]=analogRead(A0);
    reading[1]=analogRead(A1);
    reading[2]=analogRead(A2);
    reading[3]=analogRead(A3);
    reading[4]=analogRead(A4);
    reading[5]=analogRead(A5);
    reading[6]=analogRead(A6);
    for(int l=0; l<7; l++)
    {
      if(l!=7){
      if(reading[l]<sen_thr) dread[l]=0;
      else dread[l]=1;
      }
      else
      {
        if(reading[l]<300) dread[l]=0;
      else dread[l]=1;
      }
      }
      }
    
    void calc_error(){
      sum=0;
    for(int l=0; l<7;l++)
      sum=sum+dread[l];
    wsum=7*dread[0]+5*dread[1]+2*dread[2]-2*dread[4]-5*dread[5]-7*dread[6];
    err=wsum/sum;
      }

  void pause(){
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,LOW);
      digitalWrite(mlf,LOW); 
      digitalWrite(mrf,LOW);
      
      analogWrite(mlp,0);
      analogWrite(mrp,0);
      }

    void go(){
      digitalWrite(mlf,HIGH);
      digitalWrite(mrf,HIGH);
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,LOW);
      
      analogWrite(mlp,mlpwm);
      analogWrite(mrp,mrpwm);
      }     

void update_node(){ //checks which type of node it is
//Serial.println("update node");
  digitalWrite(ledr,HIGH);
  digitalWrite(ledg,HIGH);
  digitalWrite(ledb,HIGH);
  sensor_read();
  calc_error();
  left= ((dread[0]==1)&&(dread[1]==1)&&(dread[2]==1)&&(dread[3]==1)&&(dread[6]==0));
  right=((dread[6]==1)&&(dread[5]==1)&&(dread[4]==1)&&(dread[3]==1)&&(dread[0]==0));
dead_end=(sum==0);
all_white=(sum==7);
node=(dead_end||all_white);
turn=(left||right);
st=false;

/*Serial.println("left Value: "+ left );
Serial.println("right Value: "+ right );
Serial.println("dead_end Value: "+ dead_end );
Serial.println("all_white Value: "+ all_white );
Serial.println("node Value: "+ node );
Serial.println("turn Value: "+ turn );*/

if(turn){
  t=millis();
  while(millis()<t+50) go();
  sensor_read();
  left=((dread[0]==1)&&(dread[1]==1)&&(dread[2]==1)&&(dread[3]==1)&&(dread[6]==0));
  right=((dread[6]==1)&&(dread[5]==1)&&(dread[4]==1)&&(dread[3]==1)&&(dread[0]==0));
  all_white=(sum==7);
  turn=(left||right);
  node=(dead_end||all_white);
  }
if(left&&!right){
  digitalWrite(ledr,LOW);
  while(dread[0])
  {
    go();
    sensor_read();
  }
  t=millis();
  while(millis()<t+30) go();
  sensor_read();
}

else if(!left&&right)
{
  digitalWrite(ledg,LOW);
  while(dread[6])
  {
    go();
    sensor_read();
  }
  tst=millis();
  while(millis()<tst+30) go();
  sensor_read();
  if(dread[3]||dread[4]||dread[5]){
  st=true;
  right=false;
  }
}
else if(dead_end) digitalWrite(ledb,LOW);
else if(all_white){
  digitalWrite(ledr,LOW);
  digitalWrite(ledg,LOW);
  t=millis();
  while((dread[0]&&dread[6]))
  {
    if(millis()>t+600) {end_dry=1;
    digitalWrite(ledr,LOW);
  digitalWrite(ledg,LOW);
  digitalWrite(ledb,LOW);
         t=millis();
         while(millis()<t+8000) pause();}
    go();
    sensor_read();
  } 
  } 
  }
      
    
    void motor(){
      derr=err-perr;
      corr=kp*err+kd*derr;
      perr=err;
     
      if(corr<0){
        mrpwm=mrs+corr;
         mlpwm=mls-corr;
        }
      else {
         mrpwm=mrs+corr;
         mlpwm=mls-corr;
         }
      constrain(mlpwm,0,255);
      constrain(mrpwm,0,255);
      if(mlpwm>254) mlpwm==254;
      else if(mrpwm>254) mrpwm==254;
      else if(mlpwm<0) mlpwm==0;
      else if(mrpwm<0) mlpwm==0;
      go();
      }

    void line_follow(){
      sensor_read();
      calc_error();
      motor();
      }

    void turn_left(int lnov){
      //Serial.println("Turning left");
      t=millis();
      while(millis()<t+lnov) 
      {
      digitalWrite(mlf,HIGH);
      digitalWrite(mrf,HIGH);
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,LOW);
      
      analogWrite(mlp,mls);
      analogWrite(mrp,mrs);
      }
      
      t=millis();
      while(!dread[0]) {
        digitalWrite(mlf,LOW);
      digitalWrite(mrf,HIGH);
      digitalWrite(mlb,HIGH);
      digitalWrite(mrb,LOW);
      
      analogWrite(mlp,mls);
      analogWrite(mrp,mrs);
      sensor_read();
        }
        sensor_read();
      while((!dread[0])){
        digitalWrite(mrf,HIGH);
        digitalWrite(mlf,LOW);
        digitalWrite(mrb,LOW);
        digitalWrite(mlb,HIGH);
        
        analogWrite(mlp,mlt-15);
        analogWrite(mrp,mrt-15);
        sensor_read();
        }
      
      /*long long int tim =millis();
      while(millis()<tim+200)
      {
        digitalWrite(mrf,LOW);
        digitalWrite(mlf,LOW);
        digitalWrite(mrb,HIGH);
        digitalWrite(mlb,HIGH);
        
        analogWrite(mlp,mlt);
        analogWrite(mrp,mrt);
      }*/
    }

    void turn_right(){
      //Serial.println("Turning right");
      t=millis();
      while(millis()<t+line_ov) {
        digitalWrite(mlf,HIGH);
      digitalWrite(mrf,HIGH);
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,LOW);
      
      analogWrite(mlp,mls);
      analogWrite(mrp,mrs);
        }

      t=millis();
      while(!dread[6]) {
        digitalWrite(mlf,HIGH);
      digitalWrite(mrf,LOW);
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,HIGH);
      
      analogWrite(mlp,mls);
      analogWrite(mrp,mrs);
      sensor_read();
        }
        sensor_read();
      while((!dread[6])){
        digitalWrite(mlf,HIGH);
        digitalWrite(mrf,LOW);
        digitalWrite(mlb,LOW);
        digitalWrite(mrb,HIGH);
        analogWrite(mlp,mlt);
        analogWrite(mrp,mrt);
        sensor_read();
        }
        /*long long int tim =millis();
      while(millis()<tim+300)
      {
        digitalWrite(mrf,LOW);
        digitalWrite(mlf,LOW);
        digitalWrite(mrb,HIGH);
        digitalWrite(mlb,HIGH);
        
        analogWrite(mlp,mlt+10);
        analogWrite(mrp,mrt);
      }
        line_follow();*/
      }

   void turn_back(){
     //Serial.println("Turning back");
      t=millis();
      while(millis()<t+300) {
        digitalWrite(mlf,HIGH);
      digitalWrite(mrf,HIGH);
      digitalWrite(mlb,LOW);
      digitalWrite(mrb,LOW);
      
      analogWrite(mlp,0);
      analogWrite(mrp,0);
        }
        t_back=millis();
      while(!dread[2]||millis()<t_back+line_ov){
        digitalWrite(mlf,LOW);
        digitalWrite(mrf,HIGH);
        digitalWrite(mlb,HIGH);
        digitalWrite(mrb,LOW);
        
        analogWrite(mlp,mlt);
        analogWrite(mrp,mrt);
        sensor_read();
        }
        long long int tim=millis();
      while(millis()<tim+200)
      {
        digitalWrite(mrf,LOW);
        digitalWrite(mlf,LOW);
        digitalWrite(mrb,HIGH);
        digitalWrite(mlb,HIGH);
        
        analogWrite(mlp,mlt);
        analogWrite(mrp,mrt+10);
      }
      }

void LT_node(){
  if(dir[i-1]==+1) dir[i]=-2;
   else if(dir[i-1]==-2) dir[i]=-1;
 else if(dir[i-1]==-1) dir[i]=+2;
 else if(dir[i-1]==+2) dir[i]=+1;
 }
  
  void RT_node(){
  if(dir[i-1]==+1) dir[i]=+2;
   else if(dir[i-1]==-2) dir[i]=+1;
 else if(dir[i-1]==-1) dir[i]=-2;
 else if(dir[i-1]==+2) dir[i]=-1;
  }

  void tb_node(){
   dir[i]=-dir[i-1];
   }
  
  void test(){
    if(dir[i-3]+dir[i-2]==0){
      dir[i-3]=dir[i-1];
      dir[i-2]=dir[i];
      dir[i-1]=0;
      dir[i]=0;
      i=i-2;
      }
    }

void dry_run(){
 dir[0]=1;
 //dist[0]=0;
 //x[0]=0;
 //y[0]=0;

 if(i==0){
  tl=millis();
 i++;
  }
  update_node();
  while(!node&&!turn&&!st) {
    update_node();
    line_follow();
    }
  if(node||turn||st){
     //test();
   if(i>2) test();
   tl=millis();
  Serial.println(millis());
    if(left){
      turn_left(line_ov);
      LT_node();
      i++;
      }
   else if(st){
    t=millis();
    while(millis()<t+250) line_follow();
    dir[i]=dir[i-1];
    i++;
    }
   else if(right){
      turn_right();
      RT_node();
      i++;}
  else if(dead_end){
      turn_back();
      tb_node();
       i++;}
  else if(all_white){
        t=millis();
        while(millis()<t+line_ov-3){
        digitalWrite(mlf,HIGH);
        digitalWrite(mrf,HIGH);
        digitalWrite(mlb,LOW);
        digitalWrite(mrb,LOW);
        analogWrite(mlp,mls);
        analogWrite(mrp,mrs);
        }
        update_node();
   if(!end_dry){
  turn_left(0);
  LT_node();
   i++;
   }
  }
   
  }
  if(!end_dry){
    t=millis();
  digitalWrite(ledr,HIGH);
  digitalWrite(ledg,HIGH);
  digitalWrite(ledb,HIGH);
  while(millis()<t+700){
  digitalWrite(ledr,LOW);
  digitalWrite(ledg,LOW);
  digitalWrite(ledb,LOW);
    line_follow();}
  digitalWrite(ledr,HIGH);
  digitalWrite(ledg,HIGH);
  digitalWrite(ledb,HIGH);
  }
}

void follow_array(){
  if(((dir[l]==-2)&&(dir[l-1]==1))||((dir[l]==2)&&(dir[l-1]==-1))||((dir[l]==1)&&(dir[l-1]==2))||((dir[l]==-1)&&(dir[l-1]==-2))) turn_left(line_ov);
  else if(((dir[l-1]==-2)&&(dir[l]==1))||((dir[l-1]==2)&&(dir[l]==-1))||((dir[l-1]==1)&&(dir[l]==2))||((dir[l-1]==-1)&&(dir[l]==-2))) turn_right();
  else if(!(dir[l]-dir[l-1])){
    t=millis();
    while(millis()<t+250) line_follow();
  }}

void main_run(){
  line_follow();
   update_node();
  while(!node&&!turn&&!st) {
    update_node();
    line_follow();
  }
  l++;
  follow_array();
  t=millis();
  while(millis()<t+400) line_follow();
}
/*new 
 * up=+1
 * left=-2
 * down=-1
 * right=+2
*/
   void loop()
{
   if(end_dry!=1)
  dry_run();
  else
  main_run();
}
