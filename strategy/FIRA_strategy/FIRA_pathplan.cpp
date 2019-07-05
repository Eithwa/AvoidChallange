#include "FIRA_pathplan.h"
#include "math.h"
#include "time.h"

//=========Environment init=============//
FIRA_pathplan_class::FIRA_pathplan_class(){
    opponent = false;


}
//start---simulator---
void FIRA_pathplan_class::setEnv(Environment iEnv){
    env = iEnv;
    if(opponent){
        for(int i = 0;i < PLAYERS_PER_SIDE;i++){
            env.home[i] = iEnv.opponent[i];
            env.opponent[i] = iEnv.home[i];
        }
    }
}
//end  ---simulator---
void FIRA_pathplan_class::motor_place(int v,double num,int r_num){
    num_change=num*3-270;//                 60
    go_where_x=v*cos(num_change*deg2rad);// 50               70
    go_where_y=v*sin(num_change*deg2rad);
    env.home[r_num].v_x = -go_where_x;
    env.home[r_num].v_y =-go_where_y;
    env.home[r_num].v_yaw=0;
//    if((9<env.home[r_num].FB_yaw)&&(env.home[r_num].FB_yaw<180))
//    {env.home[r_num].v_yaw = -2;}
//    else if((180<env.home[r_num].FB_yaw)&&(env.home[r_num].FB_yaw<351)){env.home[r_num].v_yaw = 2;}
//    else{env.home[r_num].v_yaw=0;}
}
//車頭追球
void FIRA_pathplan_class::strategy_head2ball(int i){
    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[i].pos;
    // double robot_rotation = env.home[0].v_yaw;
    double robot_rotation = env.home[i].rotation;
    double x = ball.x - robot.x;
    if (x==0) x= very_small;
    double y = ball.y - robot.y;
    double a = atan2(y,x)*rad2deg; //*/
    //double a = 120;
    //if(x<0)       a = a-180;
    double        angle = a - robot_rotation;
    if(angle<-180)angle = angle+360;
    if(angle>180)angle = angle-360;
    env.home[i].v_yaw = angle*5;
}
//跑定點
void FIRA_pathplan_class::strategy_dst(double target_x,double target_y){
    //printf("dstX=%lf,dstY=%lf\n",dstX,dstY);
    double v_x = target_x - env.home[0].pos.x;
    double v_y = target_y - env.home[0].pos.y;
    env.home[0].v_x = v_x;
    env.home[0].v_y = v_y;
    //env.home[0].v_yaw = angle;
}
//車頭朝球＋走定點
void FIRA_pathplan_class::strategy_dst_head2ball(double target_x,double target_y){

    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[2].pos;

    double x = target_x - robot.x;
    double y = target_y - robot.y;
    double angle;
    double robot_rotation = env.home[2].rotation;
    double tar_dis = sqrt(x*x + y*y);
    double tar_ang = atan2(y,x)*rad2deg;
    double v,w;

    double x_ball = ball.x - robot.x;
    if (x_ball==0) x_ball = very_small;
    double y_ball = ball.y - robot.y;

    //v = 0.2;//1 * (1-exp(-distance/3));
    //a = atan(y/x) / pi * 180;
    double a = atan2(y_ball,x_ball) * rad2deg;
    //if(x_ball<0) a = a - 180;  //??
    angle = a - robot_rotation ;

    if(angle<-180) angle += 360;
    if(angle>=180) angle -= 360;

    //if(fabs(angle)==0)angle = very_small;

    //---speed planning---
    if(tar_dis < 0.1){
        v=0;
        if(fabs(angle) < 5) w=0;
        else w = angle/fabs(angle) * 200 * (1-exp(-1*fabs(angle)/10))+10;  //angular velocity
    }else{
        v = 250 * (1-exp(-1*tar_dis/10)) +10; //velocity
        w = 0;
    }
    env.home[2].v_x = x;
    env.home[2].v_y = y;
    env.home[2].v_yaw = w;//angle;
}

void FIRA_pathplan_class::personalStrategy(int robotIndex,int action){
    switch(action){
    case action_AvoidBarrier:
        strategy_AvoidBarrier(robotIndex);
        break;
    case Role_Halt:
        strategy_Halt(robotIndex);
        break;
    }
}
//=========================避障挑戰賽=================================
void FIRA_pathplan_class::strategy_AvoidBarrier(int Robot_index){
    int r_number = Robot_index;
    double FB_x = env.home[Robot_index].FB_x;
    double FB_y = env.home[Robot_index].FB_y;
    double FB_imu = env.home[Robot_index].FB_yaw;
    //-------------Distant-----------//
    double close_dis = Distant[0];//60 speed (10) //54  speed (30,10)
    double halfclose_dis = Distant[1];//80
    double far_dis=Distant[2];//200

    static int RedLine = 0;
    ////主要向量範圍
    static int main_vec=60;

    ///////redline
    static double fb_error=0;
    static double FB_XX=0;
    static double count=0;
    FB_XX=(fabs(FB_x-fb_error)>0.1)?FB_XX:FB_XX+FB_x-fb_error;
    if(fabs(FB_x-fb_error)<0.1){std::cout<<"fuckfuckfuckfuckfuckfuck"<<main_vec<<"\n"<<FB_imu<<"\n";}
    fb_error=FB_x;
    int r_place_x = -FB_XX*100;
    r_place_x= (r_place_x==0)?very_small:r_place_x;
    //  main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;

    //////
    static int main_go_cont=99;
    if((RedLine!=0)||(main_go_cont<13)){//speed17 sec20 speed25 sec15
        if(RedLine==1){
            main_vec=80;
            main_go_cont=0;
        }
        else if(RedLine==2){
            main_vec=40;
            main_go_cont=0;
        }else{
            main_go_cont++;
        }
    }else{
        main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;
       // main_vec=60;
    }
    /////////////////////////main vector
    main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;
    //<<<<<<<<<<<<<<<<<<<<<HEAD  Outer dynamic window<<<<<<<<<<<<<<<<<<<<<
    int mainRight=(main_vec+20>90)?90:main_vec+20;
    int mainLeft=(main_vec-20<30)?30:main_vec-20;
    int Boj_place[10][2]={0},Ok_place[30][2];
    int line_cont_b=99,line_cont_ok=99,b_ok=1,continuedline_ok=0,continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    int HowManyBoj=0,HowManyOk=0;
    #define close_oj_ignore 4
    //////////////////////////////////////////////////////2
    for(int i= mainLeft ; i<=mainRight ; i++){
        b_ok=((env.blackdis[i] <= far_dis)&&(env.blackdis[i] >= halfclose_dis)||(env.reddis[i]<=far_dis))?0:1;
        if(b_ok==1){
            continuedline_b=0;
            if(continuedline_ok==0){
                continuedline_ok=1;
                HowManyOk++;
                line_cont_ok=1;
                Ok_place[HowManyOk][0]=i;
                Ok_place[HowManyOk][1]=i;
            }else{
                line_cont_ok++;
                Ok_place[HowManyOk][1]=i;
            }
            if(line_cont_b<close_oj_ignore){
                HowManyOk--;
                if(HowManyOk==0){
                    HowManyOk=1;Ok_place[HowManyOk][0]=mainLeft;
                }
                HowManyBoj--;
                Ok_place[HowManyOk][1]=i;
                line_cont_ok=Ok_place[HowManyOk][1]-Ok_place[HowManyOk][0]+1;
                line_cont_b=99;
            }
        }else{
            continuedline_ok=0;
            if(continuedline_b==0){
                continuedline_b=1;
                HowManyBoj++;
                line_cont_b=1;
                Boj_place[HowManyBoj][0]=i;
                Boj_place[HowManyBoj][1]=i;
            }else{
                line_cont_b++;
                Boj_place[HowManyBoj][1]=i;
            }
            if(line_cont_ok<close_oj_ignore){
                HowManyBoj--;
                if(HowManyBoj==0){
                    HowManyBoj=1;Boj_place[HowManyBoj][0]=mainLeft;
                }
                HowManyOk--;
                Boj_place[HowManyBoj][1]=i;
                line_cont_b=Boj_place[HowManyBoj][1]-Boj_place[HowManyBoj][0]+1;
                line_cont_ok=99;
            }
        }
    }
    int more_ok_line=0,save_ok_line=0,right_ok=0;
    for(int i=1 ; i<=HowManyOk ;i++){
        save_ok_line=Ok_place[i][1]-Ok_place[i][0];
        if(save_ok_line>more_ok_line){
            more_ok_line=save_ok_line;
            right_ok=i;
        }
    }
    df_1=Ok_place[right_ok][1];
    df_2=Ok_place[right_ok][0];

    far_good_angle=(right_ok==0)?90:(df_1+df_2)/2;
    far_good_angle=(far_good_angle+main_vec)/2;
    ///////////////////////////////////////////////////2222222222222
    //>>>>>>>>>>>>>>>>>>>>>END   Outer dynamic window>>>>>>>>>>>>>>>>>>>>>
    //<<<<<<<<<<<<<<<<<<<<<HEAD  Inner dynamic window<<<<<<<<<<<<<<<<<<<<<
    //////////////////////////////////////////////////1
    line_cont_b=99;line_cont_ok=99;b_ok=1;continuedline_ok=0;continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    HowManyBoj=0;HowManyOk=0;

    mainRight=(int)(far_good_angle+20>90)?90:far_good_angle+20;
    mainLeft=(int)(far_good_angle-20<30)?30:far_good_angle-20;
    far_good_angle=main_vec;
    mainRight=(int)((main_vec+25)>90)?90:main_vec+25;
    mainLeft=(int)((main_vec-25)<30)?30:main_vec-25;
    // if(main_vec==40){mainRight=60;mainLeft=30;}
    // if(main_vec==80){mainRight=90;mainLeft=60;}

    for(int i= mainLeft ; i<=mainRight ; i++){
        b_ok=((env.blackdis[i] <= halfclose_dis)||(env.reddis[i]<=250))?0:1;
        if(b_ok==1){
            continuedline_b=0;
            if(continuedline_ok==0){
                continuedline_ok=1;
                HowManyOk++;
                line_cont_ok=1;
                Ok_place[HowManyOk][0]=i;
                Ok_place[HowManyOk][1]=i;
            }else{
                line_cont_ok++;
                Ok_place[HowManyOk][1]=i;
            }
            if(line_cont_b<close_oj_ignore){
                HowManyOk--;
                if(HowManyOk==0){
                    HowManyOk=1;Ok_place[HowManyOk][0]=mainLeft;
                }
                HowManyBoj--;
                Ok_place[HowManyOk][1]=i;
                line_cont_ok=Ok_place[HowManyOk][1]-Ok_place[HowManyOk][0]+1;
                line_cont_b=99;
            }
        }else{
            continuedline_ok=0;
            if(continuedline_b==0){
                continuedline_b=1;
                HowManyBoj++;
                line_cont_b=1;
                Boj_place[HowManyBoj][0]=i;
                Boj_place[HowManyBoj][1]=i;
            }else{
                line_cont_b++;
                Boj_place[HowManyBoj][1]=i;
            }
            if(line_cont_ok<close_oj_ignore){
                HowManyBoj--;
                if(HowManyBoj==0){
                    HowManyBoj=1;Boj_place[HowManyBoj][0]=mainLeft;
                }
                HowManyOk--;
                Boj_place[HowManyBoj][1]=i;
                line_cont_b=Boj_place[HowManyBoj][1]-Boj_place[HowManyBoj][0]+1;
                line_cont_ok=99;
            }
        }
    }
    more_ok_line=0;save_ok_line=0;right_ok=0;
    printf("=====================================\nhowmany_ok%d\n",HowManyOk);
    //print the obj and ok place
    for(int i=1 ; i<=HowManyOk ;i++){
        int ok_angle_text = (Ok_place[i][0]+Ok_place[i][1])/2;
        printf("ok=%d,angle=%d,dis=%d\t",i,ok_angle_text,env.blackdis[ok_angle_text]);
        std::cout<<Ok_place[i][1]<<"\t"<<Ok_place[i][0]<<"\n";
    }
    int BoxInFront=0;//0=no 1=on
    for(int i=1 ; i<=HowManyOk ;i++){
        save_ok_line=Ok_place[i][1]-Ok_place[i][0];
        if(save_ok_line>=more_ok_line){
            if((HowManyBoj==1)&&(save_ok_line-more_ok_line<=2)){
                BoxInFront=1;

            }
            more_ok_line=save_ok_line;
            right_ok=i;
        }
    }
    static int intoflag=0,tem_right_ok=0,okokcont=0,two_ok_right=60,b_goodangle=60;
    int near_angle=999,test_angle;
    dd_1=Ok_place[right_ok][1];
    dd_2=Ok_place[right_ok][0];
    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
    //>>>>>>>>>>>>>>>>>>>>>END   Inner dynamic window>>>>>>>>>>>>>>>>>>>>>
    two_ok_right=good_angle;
    if(intoflag==1){
        printf("qpqpqpqpqpqpqpqpqpqpqpqpqpqp\n");
        intoflag=(count-okokcont<14)?1:0;
        for(int i=1;i<=HowManyOk ;i++){
            dd_1=Ok_place[i][1];
            dd_2=Ok_place[i][0];
            test_angle=(int)(i==0)?90:(dd_1+dd_2)/2;
            std::cout<<test_angle<<"\t"<<tem_right_ok<<"\n";
            if(near_angle>abs(test_angle-tem_right_ok)){
                near_angle=abs(test_angle-tem_right_ok);
                good_angle=test_angle;
                std::cout<<near_angle<<"/////"<<good_angle<<"\n";
            }
        }
    }else if((abs((int)good_angle-b_goodangle)>28)&&(intoflag==0)){
        if(main_vec<60){
            for(int i=1;i<=HowManyOk ;i++){
                if(Ok_place[i][1]-Ok_place[i][0]>5){
                    right_ok=i;
                    printf("qqqqqqqqqqqqqqq\n");
                    intoflag=1;
                    okokcont=count;
                    dd_1=Ok_place[right_ok][1];
                    dd_2=Ok_place[right_ok][0];
                    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
                    tem_right_ok=good_angle;
                    break;
                }
                tem_right_ok=good_angle;
           }
       }else{
           for(int i=HowManyOk;i>=1 ;i--){
                if(Ok_place[i][1]-Ok_place[i][0]>5){
                    right_ok=i;
                    printf("ppppppppppppp\n");
                    intoflag=1;
                    okokcont=count;
                    dd_1=Ok_place[right_ok][1];
                    dd_2=Ok_place[right_ok][0];
                    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
                    tem_right_ok=good_angle;
                    break;
                }
                tem_right_ok=good_angle;
            }
       }
      // good_angle=b_goodangle;
    }
    b_goodangle=two_ok_right;

    dd_1=Ok_place[right_ok][1];
    dd_2=Ok_place[right_ok][0];
    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;

    // if(BoxInFront==1){
    //     if(env.blackdis[(int)good_angle]<60){
    //         good_angle=(main_vec<60)?30:90;
    //     }                 
    //     printf("qqqqqqqqqqqqqqq\n");
    // }
    /// //////////////////////////////////////////////11111111111111
    ///////////////////////////////////////////////////s
    line_cont_b=99;line_cont_ok=99;b_ok=1;continuedline_ok=0;continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    HowManyBoj=0;HowManyOk=0;
    int ssm_r,ssm_l;
    if(main_vec==40){ssm_r=20;ssm_l=95;}else if(main_vec==80){ssm_r=25;ssm_l=100;}else{ssm_r=25;ssm_l=95;}
    for(int i= ssm_r ; i<=ssm_l ; i++){
        b_ok=(env.blackdis[i] <= close_dis+20)?0:1;
        if(b_ok==1){
            continuedline_b=0;
            if(continuedline_ok==0){
                continuedline_ok=1;
                HowManyOk++;
                line_cont_ok=1;
                Ok_place[HowManyOk][0]=i;
                Ok_place[HowManyOk][1]=i;
            }else{
                line_cont_ok++;
                Ok_place[HowManyOk][1]=i;
            }
            if(line_cont_b<1){
                HowManyOk--;
                if(HowManyOk==0){
                    HowManyOk=1;Ok_place[HowManyOk][0]=mainLeft;
                }
                HowManyBoj--;
                Ok_place[HowManyOk][1]=i;
                line_cont_ok=Ok_place[HowManyOk][1]-Ok_place[HowManyOk][0]+1;
                line_cont_b=99;
            }
        }else{
            continuedline_ok=0;
            if(continuedline_b==0){
                continuedline_b=1;
                HowManyBoj++;
                line_cont_b=1;
                Boj_place[HowManyBoj][0]=i;
                Boj_place[HowManyBoj][1]=i;
            }else{
                line_cont_b++;
                Boj_place[HowManyBoj][1]=i;
            }
            if(line_cont_ok<close_oj_ignore){
                HowManyBoj--;
                if(HowManyBoj==0){
                    HowManyBoj=1;Boj_place[HowManyBoj][0]=mainLeft;
                }
                HowManyOk--;
                Boj_place[HowManyBoj][1]=i;
                line_cont_b=Boj_place[HowManyBoj][1]-Boj_place[HowManyBoj][0]+1;
                line_cont_ok=99;
            }
        }
    }
    ///////////////////////////////////////////////////ssssssssssssssss
    printf("dis[%d]=%d\n",60,env.blackdis[60]);
    printf("howmany_b%d\n",HowManyBoj);

    for(int i=1 ; i<=HowManyBoj ;i++){
        int Obj_angle_text = (Boj_place[i][0]+Boj_place[i][1])/2;
        printf("Boj=%d,angle=%d,dis=%d\t",i,Obj_angle_text,env.blackdis[Obj_angle_text]);
        std::cout<<Boj_place[i][1]<<"\t"<<Boj_place[i][0]<<"\n";
    }
    ////////////////////////////////////////////////////////////test for strategy
    int left_dis_sum = 0;
    int left_dis_average = 0;
    int left_average_line = 0;
    int right_dis_sum = 0;
    int right_dis_average = 0;
    int right_average_line = 0;
    int forward_dis_sum = 0;
    int forward_dis_average = 0;
    int forward_average_line = 0;
    int smallfront=999;
    static int b_forward_dis_sum=0;
    for(int i= 25 ; i<=40 ; i++){//left_dis_average
        if(env.blackdis[i]<50){
            left_average_line++;
            left_dis_sum+=env.blackdis[i];
        }
    }
    if(left_average_line > 3){ //at least 2 lines
        left_dis_average = left_dis_sum/(left_average_line);
    }else{
        left_dis_average=999;
    }
    printf("left_dis_average=%d\n",left_dis_average);
    for(int i= 80 ; i<=95 ; i++){//right_dis_average
        if(env.blackdis[i]<50){
            right_average_line++;
            right_dis_sum+=env.blackdis[i];
        }
    }
    if(right_average_line > 3){ //at least 2 lines
        right_dis_average = right_dis_sum/(right_average_line);
    }else{
        right_dis_average=999;
    }
    printf("right_dis_average=%d\n",right_dis_average);

    for(int i=55 ; i<=65 ;i++){
        if(env.blackdis[i]<200){
            forward_average_line++;
            forward_dis_sum+=env.blackdis[i];
        }
        smallfront=(smallfront<env.blackdis[i])?smallfront:env.blackdis[i];
    }
    if(forward_average_line > 3){ //at least 4 lines
        forward_dis_average = forward_dis_sum/(forward_average_line);
    }else{
        forward_dis_average=999;
    }

    printf("forward_dis_average=%d\n",forward_dis_average);
    double adjust_ojF=1;
    int dis_sum = 0;
    int dis_average = 0;
    int red_dis_average = 0;
    int angle_average;
    int dangerous_dis =(int)close_dis;
    double Fx,Fy = 0;
    int F_Max = (int)close_dis;
    int Obj_angle,ignore_average_line=0 ;
    int condition;//0::0引力  1::1special box_middle
    int red_line_dangerous_dis =70;
    int small_dis_box=999;
    double final_Fx = 0;
    double final_Fy = 0;
    RedLine = 0;

    left_average_line = 0;
    right_average_line = 0;
    for(int i= 25 ; i<=40 ; i++){//left_dis_average
        if(env.reddis[i]<red_line_dangerous_dis){
            left_average_line++;
            dis_sum+=env.reddis[i];
        }
    }
    if(left_average_line > 3){ //at least 2 lines
        red_dis_average = dis_sum/(left_average_line);
        RedLine = 1;//red in left
        FB_XX=(red_dis_average!=0)?(red_dis_average-150)*0.01:FB_XX;
    }
    dis_sum=0;

    for(int i= 80 ; i<=95 ; i++){//right_dis_average
        if(env.reddis[i]<red_line_dangerous_dis){
            right_average_line++;
            dis_sum+=env.reddis[i];
        }
    }
    if(right_average_line > 3){ //at least 2 lines
        red_dis_average = dis_sum/(right_average_line);
        RedLine = 2;//red in right
        FB_XX=(red_dis_average!=0)?(150-red_dis_average)*0.01:FB_XX;
    }
    dis_sum=0;
    if(env.reddis[90]<100){
        FB_XX=(env.reddis[90]!=0)?(150-env.reddis[90])*0.01:FB_XX;
        printf("gooooodredline\n");
    }
    if(env.reddis[30]<100){
        FB_XX=(env.reddis[30]!=0)?(env.reddis[30]-150)*0.01:FB_XX;
        printf("redlinegooood\n");
    }

    if(left_dis_average+right_dis_average<=90){
        condition=box_in_between;
    }/*else if((smallfront <=42)&&(main_vec==40||main_vec==80)){
        condition = red_line;
    }*//*else if (intoflag==1||intoflag==2){condition=cannot_chose;}*/
    else{
        condition=N_S;
    }

    count+=1;
    static double before_error_x=0,I_error_x=0;
    static double Fx_pid,Fy_pid,PID_I_tem[20]={0} ;
    double error_x,D_error;
   // condition=pid_control;
    switch(condition){
    case N_S:
        for(int i=1 ; i<=HowManyBoj ;i++){ //repulsive force
            for(int j=Boj_place[i][0] ; j<=Boj_place[i][1] ; j++){
                if(env.blackdis[j]>=close_dis){
                    ignore_average_line++;
                }else{
                    dis_sum+=env.blackdis[j];
                }
               small_dis_box=(small_dis_box<env.blackdis[j])?small_dis_box:env.blackdis[j];
            }
            if(Boj_place[i][1]-Boj_place[i][0]+1!=ignore_average_line){
                dis_average = dis_sum/(Boj_place[i][1]-Boj_place[i][0]+1-ignore_average_line);
            }else{
                dis_average=999;
            }
            dis_average=small_dis_box;
      //      dis_average=(dis_average<33)?0:dis_average;
            small_dis_box=999;
            ignore_average_line=0;

            if(dis_average<=dangerous_dis){
                Obj_angle = (Boj_place[i][0]+Boj_place[i][1])/2;
                printf("warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
                angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
                if(((Obj_angle<30)||(Obj_angle>90))/*&&((main_vec!=80)&&(main_vec!=40))*/){
                Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad*0.8);//[53]->angle 53
                Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad)*0.8;}else{
                Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
                Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
                }
                printf("Fx=%lf,Fy=%lf\n",Fx,Fy);
            }
            dis_sum=0;
            dis_average=999;
        }
        if(RedLine!=0){
            printf("RedLine=%d\tred_dis_average=%d\n",RedLine,red_dis_average);
            if(RedLine==2){
                red_dis_average=red_line_dangerous_dis-red_dis_average;
            }else{
                red_dis_average=red_dis_average-red_line_dangerous_dis;
            }
        }
        final_Fx = F_Max*cos((90-(good_angle))*3*deg2rad)-adjust_ojF*Fx-(0.2*red_dis_average);
        final_Fy = F_Max*sin((90-(good_angle))*3*deg2rad)-adjust_ojF*Fy;
        final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;
        printf("S/N========  ");
        break;
    case box_in_between:
        if(left_dis_average < right_dis_average){
            final_angle = 90-(90-(right_dis_average-left_dis_average))/3;//turn right
        }else if(left_dis_average > right_dis_average){
            final_angle = 90-(90+(left_dis_average-right_dis_average))/3;//turn left
        }else{
            final_angle = 60;//go forward
        }
        printf("middle box========  ");
        break;
    case red_line:
        if(main_vec == 40){
            if(b_forward_dis_sum-smallfront<0){final_angle = 29;}
            else if(b_forward_dis_sum-smallfront>0){final_angle = 31;}
            else{final_angle=30;}
        }else if(main_vec == 80){
                if(b_forward_dis_sum-smallfront<0){final_angle = 91;}
                else if(b_forward_dis_sum-smallfront>0){final_angle = 89;}
                else{final_angle=90;}
        }
        printf("red front========  ");
        b_forward_dis_sum=smallfront;
        break;
    case pid_control:
        before_error_x=Fx_pid;
        Fx_pid=0;Fy_pid=0;
        for(int i=1 ; i<=HowManyBoj ;i++){ //repulsive force
            for(int j=Boj_place[i][0] ; j<=Boj_place[i][1] ; j++){
                if(env.blackdis[j]>=close_dis){
                    ignore_average_line++;
                }else{
                    dis_sum+=env.blackdis[j];
                }
            }
            if(Boj_place[i][1]-Boj_place[i][0]+1!=ignore_average_line){
                dis_average = dis_sum/(Boj_place[i][1]-Boj_place[i][0]+1-ignore_average_line);
            }else{
                dis_average=999;
            }
            ignore_average_line=0;
                Obj_angle = (Boj_place[i][0]+Boj_place[i][1])/2;
                printf("warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
                angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
                Fx_pid += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
                Fy_pid += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
            dis_sum=0;
        }
        I_error_x=0;
            PID_I_tem[(int)count%20]=Fx_pid;
            for(int i=0;i<19;i++){
                I_error_x+=PID_I_tem[i];
            }
            I_error_x=I_error_x/20;
            D_error=Fx_pid-before_error_x;
       error_x =kd*(D_error)+ki*(I_error_x)+kp*Fx_pid;

       final_Fx = F_Max*cos((90-(60))*3*deg2rad)-error_x;
       final_Fy = F_Max*sin((90-(/*good_angle*/60))*3*deg2rad)-adjust_ojF*Fy;
       final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;
          printf("\nkp=%lf,\nki=%lf\nkd%lf\nerror_x%lf\nI_error_x%lf\nD_error%lf\nP_error%lf\n",kp,ki,kd,error_x,I_error_x,D_error,Fx_pid);
        break;

    }
    /////////////////////////////////////////////////////////////////////////////////////
    printf("final_angle=%lf\n",final_angle);
    static int v_fast=50;
    static int b_not_good_p=0;
    static int stop_count=0;
    if((int)count%5==1){
        if(b_not_good_p==not_good_p){ 
            v_fast=v_fast+10;
            v_fast=(v_fast<100)?v_fast:100;
        } else{ 
            v_fast= v_fast-10;
            v_fast=(v_fast>1)?v_fast:1;   
        }
        if((forward_dis_average>100)&&(HowManyBoj<=1)&&((40<main_vec)&&(main_vec<80))/*||(condition==box_in_between)*/){
            v_fast=v_fast+30;v_fast=(v_fast<100)?v_fast:100;
        }else{
            v_fast=1;
            v_fast=(v_fast>1)?v_fast:1;
            if(condition==box_in_between){
                v_fast=1;
            }
        }
    }
    if(b_not_good_p==not_good_p-1){
       stop_count++;
       if(stop_count>10){
           v_fast=0; 
           printf("dangerous\n\n\n\n\nno_run_blackitem\n\n\nnopicture_fuck_you\n\n\nrestrat\n\n");
        }
    }else{
        stop_count=0;
    }

    b_not_good_p=not_good_p;
    // v_fast=1;
    v_fast=(avoid_go==0)?0:v_fast;
    FB_XX=(avoid_go==0)?0:FB_XX;
    motor_place(v_fast,final_angle,r_number);
    // vision_per.good_angle=good_angle;
    // vision_per.final_angle=final_angle;
    // vision_per.far_good_angle=far_good_angle;
    // vision_per.dd_1=dd_1;
    // vision_per.dd_2=dd_2;
    // vision_per.df_1=df_1;
    // vision_per.df_2=df_2;
    // tovision.publish(vision_per);
    printf("v_fast=%d\n",v_fast);
    printf("ave_gray=%d\n",env.gray_ave);
    // printf("not_good_p=%d\n",not_good_p);
    // printf("30_dis=%d\n",env.blackdis[30]);
    // printf("90_dis=%d\n",env.blackdis[90]);
    printf("count=%lf\n",count);
    printf("main_vec=%d\n",main_vec);
    printf("far_good_angle=%f\n",far_good_angle);
    printf("good_angle=%f\n",good_angle);
    printf("final_angle=%lf\n",final_angle);
    printf("FB_x=%lf\t,FB_y=%lf\t",FB_x,FB_y);
    printf("FB_xx=%lf\t,FB_err=%lf\t\n",FB_XX,fb_error);
    // printf("FB_x=%lf\t,FB_y=%lf\t,FB_imu=%lf\n",FB_x,FB_y,FB_imu);
    // printf("MotorAngle=%lf\n",num_change);
}
//=========================避障挑戰賽結束=================================
void FIRA_pathplan_class::strategy_Halt(int Robot_index){
    env.home[Robot_index].v_x = 0;
    env.home[Robot_index].v_y = 0;
    env.home[Robot_index].v_yaw = 0;
}

double FIRA_pathplan_class::vecAngle(Vector2d a,Vector2d b){

    a.normalize();
    b.normalize();
    double c = a.dot(b);
    double rad = acos(c);


    Vector3d a3d = Vector3d(a(0),a(1),0);
    Vector3d b3d = Vector3d(b(0),b(1),0);

    Vector3d tCross = a3d.cross(b3d);

    if(tCross(2) < 0)       return (-1)*rad*rad2deg;
    else                    return rad*rad2deg;
}

//==========for ROS special===============//

void FIRA_pathplan_class::loadParam(ros::NodeHandle *n){
   tovision  = n->advertise<strategy::strategylook>("/vision/strategy_line",1);
    n->setParam("/AvoidChallenge/GoodAngle",good_angle);
    n->setParam("/AvoidChallenge/FinalAngle",final_angle);
    n->setParam("/AvoidChallenge/dd_1",dd_1);
    n->setParam("/AvoidChallenge/dd_2",dd_2);
    n->setParam("/AvoidChallenge/far_good_angle",far_good_angle);
    n->setParam("/AvoidChallenge/df_1",df_1);
    n->setParam("/AvoidChallenge/df_2",df_2);
    n->getParam("/AvoidChallenge/kp",kp);
    n->getParam("/AvoidChallenge/ki",ki);
    n->getParam("/AvoidChallenge/kd",kd);
    n->getParam("/AvoidChallenge/avoid_go",avoid_go);
    not_good_p=(env.picture_m==b_picture_m)?not_good_p+1:not_good_p;
    b_picture_m=env.picture_m;
    if(n->getParam("/AvoidChallenge/Distant", Distant)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
  /*  if(n->getParam("/AvoidChallenge/Line", ScanLine)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/AvoidChallenge/ChooseSide", ChooseSide)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Attack_Strategy", Attack_Strategy)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Chase_Strategy", Chase_Strategy)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Zone_Attack", Zone_Attack)){
        //        for(int i=0;i<2;i++)
        //            std::cout<< "param Zone_Attack["<< i << "]=" << Zone_Attack[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/TypeS_Attack", TypeS_Attack)){
        //       for(int i=0;i<2;i++)
        //           std::cout<< "param TypeS_Attack["<< i << "]=" << TypeS_Attack[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/TypeU_Attack", TypeU_Attack)){
        //       for(int i=0;i<8;i++)
        //           std::cout<< "param TypeU_Attack["<< i << "]=" << TypeU_Attack[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/SideSpeedUp", SideSpeedUp)){
        //       for(int i=0;i<5;i++)
        //           std::cout<< "param SideSpeedUp["<< i << "]=" << SideSpeedUp[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }*/
}

