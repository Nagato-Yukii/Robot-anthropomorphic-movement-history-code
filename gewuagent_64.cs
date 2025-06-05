using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using Unity.Sentis;

public class GewuAgent : Agent
{
    int tp = 0;
    int tq = 0;
    int tt = 0;
    int right_first;
    public bool fixbody = false;
    public bool train;
    public bool accelerate;
    float uff = 0;
    float uf1 = 0;
    float uf2 = 0;
    float uf1_time = 0;
    float uf2_ = 0;
    float uf1_ = 0;
    float uf2_time = 0;
    int change = 1;
    int flag1 = 0;
    int flag2 = 0;
    //float b_balance = 10;
    float b_num = 10;
    int signal = 1;
    // u = new float[ActionNum]; ActionNum = 23(G1)
    float[] u = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] ut = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] utt = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] utotal = new float[23]{0,0,0,90,0,0,0,0,90,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    int T1 = 50;
    int T2 = 30;
    int tp0 = 0;
    public enum StyleB
    {
        walk = 0,
        run = 1,
        jump = 2
    }
    public enum StyleQ
    {
        trot = 0,
        bound = 1,
        pronk = 2,
        march = 3
    }
    public enum StyleL
    {
        drive = 0,
        walk = 1,
        jump = 2
    }
    public enum StyleR
    {
        biped = 0,
        quadruped = 1,
        legwheeled = 2
    }
    //transform储存组件的位置和姿态信息,每个gameobject都有一个transform组件,body存储了机器人躯干?的transform组件
    Transform body;
    public int ObservationNum;
    public int ActionNum;
    //show on inspector
    [Header("RobotType")]
    public StyleR Robot;
    [Header("Biped")]
    public StyleB BipedTargetMotion;
    public ModelAsset BWalkPolicy;
    public ModelAsset BRunPolicy;
    public ModelAsset BJumpPolicy; 
    [Header("Quadruped_G1")]
    public StyleQ QuadrupedTargetMotion;
    public ModelAsset QTrotPolicy;
    public ModelAsset QBoundPolicy;
    public ModelAsset QPronkPolicy;
    public ModelAsset QMarchPolicy;
    [Header("Legwheeled")]
    public StyleL LegwheelTargetMotion;
    public ModelAsset LDrivePolicy;
    public ModelAsset LWalkPolicy;
    public ModelAsset LJumpPolicy;
    //声明变量,只用上一个时间的状态 ~~马尔可夫链?
    StyleB LastBmotion;
    StyleQ LastQmotion;
    StyleL LastLmotion;
    //创建了一个List<float>()赋给P0,W0,是动态大小的List,每个元素是float
    List<float> P0 = new List<float>();
    List<float> W0 = new List<float>();
    List<Transform> bodypart = new List<Transform>();
    //Unity自带的数据类型,通常表示一个三维向量
    Vector3 pos0;
    //Unity自带的数据类型,通常表示一个三维旋转(加了一个欧拉角)
    Quaternion rot0;
    //arts 应该是 Articulation的简写, acts表示动作
    //ArticulationBody[] arts = new ArticulationBody[40];
    //ArticulationBody[] acts = new ArticulationBody[16];
    ArticulationBody[] arts = new ArticulationBody[60];
    ArticulationBody[] acts = new ArticulationBody[30];
    GameObject robot;
    /*
    float[] kb = new float[16] { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30  };
    float[] kb1 = new float[16] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] kb2 = new float[16] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    */
    float[] kb = new float[23]{30,30,30,30,30,30,30,30,30,30, 30,30,30,30,30,30,30,30,30,30, 30,30,30};
    float[] kb1 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] kb2 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};

    float dh = 25;
    float d0 = 15;
    float ko = 0.5f;
    float kh = 0;
//重写了Initialize方法

public override void Initialize()
{
    // 获取当前 GameObject 及其所有子物体上所有的 ArticulationBody 组件
    //C#中,this直接指代当前脚本的实例,GetComponentsInChildren也是Unity自带的常用函数,跟着树一直查查到底
    arts = this.GetComponentsInChildren<ArticulationBody>();
    //print($"在机器人模型中找到总计 {arts.Length} 个 ArticulationBody 组件.");//G1是33个
    // 使用 List 来临时存储找到的可控关节,然后遍历找到的所有 ArticulationBody 组件 动态大小
    List<ArticulationBody> actsList = new List<ArticulationBody>();
    for (int k = 0; k < arts.Length; k++)
    {
        if (arts[k] != null)
        {   
            // 检查关节类型是否为 RevoluteJoint (旋转关节)
            //print($"arts[{k}]的关节类型是{arts[k].jointType.ToString()}");
            if(arts[k].jointType.ToString() == "RevoluteJoint")
            {
                // 如果是 RevoluteJoint,将其视为可控关节并添加到列表actsList中
                actsList.Add(arts[k]);
                //print($"找到可控关节: {arts[k].name}");
            }
        }
        else
        {
            print($"在遍历arts时,发现第arts[{k}]是null,警告: 在 arts 数组的索引 {k} 处发现一个空的 ArticulationBody 组件引用.");
        }
    }

    acts = actsList.ToArray();
    ActionNum = acts.Length;
    //print($"在机器人模型中找到 {ActionNum} 个 RevoluteJoint 作为可控动作关节.");//G1机器人为23个

    // 获取机器人身体（通常是根 ArticulationBody）的 Transform
    // 假设 arts[0] 仍然是根节点，但稳妥起见，也检查一下 arts 数组是否非空
    body = (arts != null && arts.Length > 0) ? arts[0].GetComponent<Transform>() : null;

    // 如果 body 成功获取，存储初始位置和旋转
    if (body != null)
    {
        pos0 = body.position;
        rot0 = body.rotation;
    }
    else
    {
        // 如果没有找到 ArticulationBody 或 arts[0] 是 null，打印错误
        print("错误: 未能获取机器人身体的 Transform (arts 数组为空或 arts[0] 为 null).");
        // 在这种情况下，机器人可能无法正常初始化和运行
        // 可以考虑在这里 EndEpisode() 或者采取其他错误处理
        // 如果 body 是 null，后续依赖 body 的代码也会报错
    }

    /*
    // 收集所有可控关节的初始位置和速度
    // 现在使用 acts 数组，其大小等于 ActionNum
    //关节缓存大小的计算方法是所有关节自由度的总和加上6
    for (int i = 0; i < 6; i ++)
    {
        P0.Add(0f);
        W0.Add(0f);
    }
    for (int i = 0; i < ActionNum; i++)
    {
        // 这里的 acts[i] != null 检查仍然是必要的，尽管我们之前过滤了 arts 里的 null
        if (acts[i] != null)
        {
            if (acts[i].jointPosition.dofCount > 0) P0.Add(acts[i].jointPosition[0]);
            else P0.Add(0f);

            if (acts[i].jointVelocity.dofCount > 0) W0.Add(acts[i].jointVelocity[0]);
            else W0.Add(0f);
        }
        else
        {
            // 如果 acts[i] 在这里是 null,说明之前填充 actsList 时混入了 null
            print($"错误: 在收集初始状态时 acts[{i}] 为 null. 请检查模型配置和 ArticulationBody 组件.");
            P0.Add(0f); // 添加默认值避免索引错误
            W0.Add(0f);
        }
    }
    */
        pos0 = body.position;
        rot0 = body.rotation;
        arts[0].GetJointPositions(P0);
        arts[0].GetJointVelocities(W0);
    

    // 设置加速模式，仅当 train 为 true 时才启用
    accelerate = train;

    // 只有当机器人身体成功获取且找到可控关节时，才认为初始化成功一部分
    // 否则可能会在 CollectObservations 等地方继续报错
     if (body == null || ActionNum == 0)
     {
        print("存在body == null || ActionNum == 0");
        print("Initialization failed: body not found or no action joints found.");
         // 可以选择在这里强制结束回合，避免持续报错
         // EndEpisode(); // 根据情况决定是否需要
     }
}


    private bool _isClone = false; 
    void Start()
    {
        Time.fixedDeltaTime = 0.01f;

        SerializedObject tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset"));
        SerializedProperty layers = tagManager.FindProperty("layers");
        SerializedProperty layer = layers.GetArrayElementAtIndex(15);
        int targetLayer = LayerMask.NameToLayer("robot");
        layer.stringValue = "robot";
        tagManager.ApplyModifiedProperties();
        Physics.IgnoreLayerCollision(15, 15, true);
        ChangeLayerRecursively(gameObject, 15);

        if (train && !_isClone) 
        {
            for (int i = 1; i < 14; i++)
            {
                GameObject clone = Instantiate(gameObject); 
                clone.transform.position = transform.position + new Vector3(i * 2f, 0, 0);
                clone.name = $"{name}_Clone_{i}"; 
                clone.GetComponent<GewuAgent>()._isClone = true; 
            }
        }
    }
    void ChangeLayerRecursively(GameObject obj, int targetLayer)
    {
        obj.layer = targetLayer;
        foreach (Transform child in obj.transform)ChangeLayerRecursively(child.gameObject, targetLayer);
    }

    public override void OnEpisodeBegin()
    {
        tp = 0;
        tq = 0;
        tt = 0;
        for (int i = 0; i < ActionNum; i++)u[i] = 0;
        for (int i = 0; i < ActionNum; i++)ut[i] = 0;
        for (int i = 0; i < ActionNum; i++)utt[i] = 0;

        ObservationNum = 33;//ActionNum = 23
        if (fixbody) arts[0].immovable = true;
        if (!fixbody)
        {
            arts[0].TeleportRoot(pos0, rot0);
            arts[0].velocity = Vector3.zero;
            arts[0].angularVelocity = Vector3.zero;
            arts[0].SetJointPositions(P0);
            arts[0].SetJointVelocities(W0);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(body.InverseTransformDirection(Vector3.down));
        sensor.AddObservation(body.InverseTransformDirection(arts[0].angularVelocity));
        sensor.AddObservation(body.InverseTransformDirection(arts[0].velocity));
        
        for (int i = 11; i < ActionNum; i++)
        {
            sensor.AddObservation(acts[i].jointPosition[0]);
            sensor.AddObservation(acts[i].jointVelocity[0]);
        }
        
    }
    float EulerTrans(float eulerAngle)
    {
        if (eulerAngle <= 180)
            return eulerAngle;
        else
            return eulerAngle - 360f;
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        for (int i = 0; i < 23; i++) utotal[i] = 0;
        var continuousActions = actionBuffers.ContinuousActions;
        var kk = 0.9f;
        
        for (int i = 0; i < ActionNum; i++)
        {
            u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
            ut[i] += u[i];
            utt[i] += ut[i];
            utotal[i] = kb[i] * u[i] + kb1[i] * ut[i] + kb2[i] * utt[i];
            if (fixbody) utotal[i] = 0;
        }

        string name = this.name;
        //***********************************************************************************************************************************
        int[] idx = new int[6] { 2, 3, 4, 7, 8, 9 };
        if (name.Contains("MyRobot"))
        {
            idx = new int[6] { 2, 3, 4, 7, 8, 9 };//biped with 5 joints in each leg, change here if the feedforward is incorrect 
        }
        if (Robot == StyleR.biped && ActionNum == 10)
        {
            if (BipedTargetMotion == StyleB.walk){}
            if (BipedTargetMotion == StyleB.run){}
            if (BipedTargetMotion == StyleB.jump){}
        }
        //***********************************************************************************************************************************
        idx = new int[6] { 3, 4, 5, 9, 10, 11 };
        if (name.Contains("MyRobot"))
        {
            idx = new int[6] { 3, 4, 5, 9, 10, 11 };//biped with 6 joints in each leg, change here if the feedforward is incorrect 
        }
        {
            if (BipedTargetMotion == StyleB.walk){}
            if (BipedTargetMotion == StyleB.run){}
            if (BipedTargetMotion == StyleB.jump){}
        }
        if (Robot == StyleR.legwheeled && ActionNum == 8)
        {
            if (LegwheelTargetMotion == StyleL.walk){}
            if (LegwheelTargetMotion == StyleL.jump){}
            if (LegwheelTargetMotion == StyleL.drive){}
        }
        if (Robot == StyleR.legwheeled && ActionNum == 16)
        {
            if (LegwheelTargetMotion == StyleL.walk){}
            if (LegwheelTargetMotion == StyleL.jump){}
            if (LegwheelTargetMotion == StyleL.drive){}
        }
        //***********************************************************************************************************************************
        idx = new int[12] {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };
        if (name.Contains("MyRobot"))
        {
            idx = new int[12] {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };//biped with 6 joints in each leg, change here if the feedforward is incorrect 
        }
        if (Robot == StyleR.quadruped)
        {
            //quadruped robot, modify the following parameters to optimize the gait if needed********************************************
            T1 = 70;//gait period
            dh = 15;//foot stepping height
            d0 = 18;//knee bend angle
            //***************************************************************************************************************************
            if (QuadrupedTargetMotion == StyleQ.trot){}
            if (QuadrupedTargetMotion == StyleQ.bound){}
            if (QuadrupedTargetMotion == StyleQ.pronk){}
            if (QuadrupedTargetMotion == StyleQ.march)
            {        
                T1 = 80;//gait period
                dh = 20;//foot stepping height
                d0 = 0;//knee bend angle
                ko = 0.5f;//1
                float[] ktemp = new float[23] { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 
                                                30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
                                                30, 30, 30 };//feedback ratio, represents the action space
                float[] kb =  new float[23]{0,0,0,0,0,0,0,0,0,0, 0,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num, b_num,b_num,b_num};
                float[] kb1 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num, b_num,b_num,b_num};
                float[] kb2 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num,b_num, b_num,b_num,b_num};
                for (int i = 0; i < 23; i++) kb[i] = 0.05f*ktemp[i];
                if(change == 1)
                {
                    for (int i = 0; i < utotal.Length; i++)
                    {
                        utotal[i] = 0f;
                    }
                        //print($"utotal[{9}] = {utotal[i]}");
                }        
                if(right_first == 1&&flag1 == 1)
                //右腿前抬,左腿后撤,左臂前挥,右臂后甩
                {
                    //print("get into right:stleb.march , if right_first == 1 && flag1 = 1");
                    flag1 = 0;
                    if (uf1_time != 1)
                    {
                        //print("get into uf1_time! =  1");
                        //下半身1/4*T1~1/2T1左后撤步
                        utotal[17] += (8 * (dh * uf2_ + d0) / 3 * Mathf.Sign(idx[0]) - 8 * (dh * uf2 + d0) / 3 * Mathf.Sign(idx[0]));
                    }
                    //右腿前抬又收回
                    utotal[11] -= 8 * (dh * uf1 + d0)/3 * Mathf.Sign(idx[0]); //右腿前后摆动
                    utotal[14] = utotal[11]; //膝关节
                    utotal[15] -= utotal[14];//右脚背与小腿的夹角
                    //上半身左臂前挥
                    utotal[6] -= 45*uf1;//大臂前后摆动
                    utotal[7] -= 20*uf1;//大臂略微向外曲张
                    utotal[8] -= 100*uf1;//(dh * uf2 + d0) * -2;//左小臂和左大臂的角度
                    utotal[9] += 30*uf1;
                    //上半身/右臂后甩
                    utotal[4] += 90*uf1;
                    utotal[1] += 50*uf1;
                    utotal[2] -= 20*uf1;
                    
                    //utotal[3] -= 3*(dh * uf2 + d0);
                }
                if(right_first == 0&&flag2 == 1)
                {
                    //print("get into left:stleb.march , if right_first == 0 && flag2 = 1");
                    flag2 = 0;
                    if(uf2_time != 1)
                    {
                        //print("get into uf2_time! =  1");
                        //左腿后撤，右腿回正
                        utotal[11] -= (8 * (dh * uf1_ + d0) / 3 * Mathf.Sign(idx[3]) - 8 * (dh * uf1 + d0) / 3 * Mathf.Sign(idx[3]));
                    }
                    utotal[17] -= 8 * (dh * uf2 + d0)/3 * Mathf.Sign(idx[0]); //右腿前后摆动
                    utotal[21] -= utotal[17];//右脚背与小腿的夹角
                    utotal[22] = utotal[17];
                    //上半身
                    utotal[1] -= 45*uf2;//大臂前后摆动
                    //utotal[2] += 20;//大臂略微向外曲张
                    utotal[2] -= 20*uf2;
                    utotal[3] += 100*uf2;//(dh * uf2 + d0) * -2;//左小臂和左大臂的角度
                    utotal[4] -= 20*uf2;

                        //左臂后摆
                    utotal[9] += 90*uf2;
                    utotal[6] += 50*uf2;//
                    utotal[7] += 20*uf2;
                    

                }
                if (!train) SetModel("Quadruped", QMarchPolicy);
            }
        }

        for (int i = 0; i < ActionNum; i++) SetJointTargetDeg(acts[i], utotal[i]);

        if (BipedTargetMotion != LastBmotion || QuadrupedTargetMotion != LastQmotion || LegwheelTargetMotion != LastLmotion) EndEpisode();
        LastBmotion = BipedTargetMotion;
        LastQmotion = QuadrupedTargetMotion;
        LastLmotion = LegwheelTargetMotion;
    }
    void SetJointTargetDeg(ArticulationBody joint, float x)
    {
        var drive = joint.xDrive;
        drive.stiffness = 2000f;
        drive.damping = 100f;
        drive.forceLimit = 300f;
        drive.target = x;
        joint.xDrive = drive;
    }
    void SetJointTargetPosition(ArticulationBody joint, float x)
    {
        x = (x + 1f) * 0.5f;
        var x1 = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, x);
        var drive = joint.xDrive;
        drive.stiffness = 2000f;
        drive.damping = 100f;
        drive.forceLimit = 200f;
        drive.target = x1;
        joint.xDrive = drive;
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        
    }

    void FixedUpdate()
    {

        if (accelerate) Time.timeScale = 20;
        if (!accelerate) Time.timeScale = 1;

        tp++;
        tq++;
        tt++;
        print($"signal = {signal}");
        print($"b_num = {b_num}");
        print($"t_end = {1000*signal}");

        //判断应该伸出右脚还是左脚
        if (tp > 0 && tq <= T1)
        //右腿
        {
            right_first = 1;
        }
        else
        //左腿
        {
            right_first = 0;
        }

        change = right_first;
        if( uf1 + uf2 == 0)
        {
            change = 1;
        }
        else{
            change = 0;
        }
        //右腿计算
        if (tp > 0 && tp <= T1)
        {
            tp0 = tp;
            uf1 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) / 2f;
            uf2 = 0;
            flag1 = 1;
            tp0 = tp;
            if ((float)tp < 3*(float)T1/5 ){uf1_time = 1;}
            else{uf1_time = 0;}            
            uf1 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f)/2f;
            uf2 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) /4f;
            uf2_ = (-Mathf.Cos(3.14f * 2 * 2/5) + 1f) /4f;
        }
        
 
        if (tp > T1 && tp <= 2 * T1)
        {
            tp0 = tp - T1;
            uf1 = 0;
            uf2 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) / 2f;
            right_first =0;
            flag2 = 1;
            tp0 = tp - T1;
            if ((float)tp0 < 3*(float)T1/5 ){uf2_time = 1;}
            else{uf2_time = 0;} 
            uf2 = (-Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) / 2f;
            uf1 = (Mathf.Cos(3.14f * 2 * tp0 / T1) + 1f) /4f;
            uf1_ = (Mathf.Cos(3.14f * 2 * 2/5) + 1f) /4f;
            
        }
        if (tp >= 2 * T1) tp = 0;
        uff = (-Mathf.Cos(3.14f * 2 * tq / T2) + 1f) / 2f;
        if (tq >= T2) tq = 0;
        var vel = body.InverseTransformDirection(arts[0].velocity);
        var wel = body.InverseTransformDirection(arts[0].angularVelocity);
        var live_reward = 1f;

        var ori_reward1 = -1f * Mathf.Abs(EulerTrans(body.eulerAngles[0]));
        var ori_reward2 = -10f * Mathf.Abs(wel[1]);
        var ori_reward3 = -1f * Mathf.Abs(EulerTrans(body.eulerAngles[2]));
        var vel_reward1 = (vel[2] - Mathf.Abs(vel[0]));
        var vel_reward2 = (vel[2] - Mathf.Abs(vel[0]) + kh * Mathf.Abs(vel[1]));
        ///////////////////////////////////////////////////////////////////////
        var reward = live_reward + (ori_reward1 + ori_reward2 + ori_reward3) * ko + 0.5f * vel_reward2;// + yaw_rate_penalty_calc ;//- pose_lose;// - pose_lose;
        print($"reward= {reward}");
        AddReward(reward);
        if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > 3f || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > 3f || tt>= 2000*signal)
        {
            if(train)EndEpisode();
        }
        if(reward > 5)
        {
            b_num /= 2;
            signal *= 2;
        //    print($"reward > b_balance * signal");
            print($"b_num /= 2;");
            print($"signal *= 2;");
        }
        else
        {
            print("not reach bound");
        }
    }

}