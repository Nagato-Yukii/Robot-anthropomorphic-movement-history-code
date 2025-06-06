//最后一次修改:2025/6/6 23:36
//支持在retarget中进行训练(初始版本，仍需修改奖励等),没有trainconfig直接去RL-Playground的复制到retarget文件夹下
//在只有挥手动作的情况下(wave_both15_poses),训练几分钟后，在终端还没有任何次数显示，暂停，训练结果和前馈动作相比，手臂抬升更高，抬升更慢
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

using System;
using System.Net;
using System.Text;
using System.Linq;

using UnityEditor;
using Unity.Sentis;

public class H1Agent : Agent
{
    float[] uff = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] u = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] u0 = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] utotal = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    private List<string> actionFolders = new List<string>();
    private int currentActionIndex = 0;

    private string dofFilePath;
    private string rotFilePath;
    private string posFilePath;

    private List<float[]> allDofData = new List<float[]>();
    private List<float[]> allPosData = new List<float[]>();
    private List<float[]> allRotData = new List<float[]>();
    
    private int currentFrame = 0;  
    private bool isEndEpisode = false;
    
    Transform body;
    List<float> P0 = new List<float>();
    List<float> W0 = new List<float>();
    ArticulationBody[] arts = new ArticulationBody[40];
    Vector3 pos0;
    Quaternion rot0;
    //ArticulationBody[] jh = new ArticulationBody[19];
    ArticulationBody[] jh = new ArticulationBody[19];
    ArticulationBody art0;
    public enum StyleB
    {
        walk = 0,
        run = 1,
        jump = 2
    }
    public enum StyleQ
    {
        trot = 0,
        march = 1,
        wave = 2
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
    [Header("RobotType")]
    public StyleR Robot;
    [Header("Biped")]
    public StyleB BipedTargetMotion;
    public ModelAsset BWalkPolicy;
    public ModelAsset BRunPolicy;
    public ModelAsset BJumpPolicy; 
    [Header("Quadruped_H1")]
    public StyleQ QuadrupedTargetMotion;
    public ModelAsset QTrotPolicy;
    public ModelAsset QMarchPolicy;
    public ModelAsset QWavePolicy;
    [Header("Legwheeled")]
    public StyleL LegwheelTargetMotion;
    public ModelAsset LDrivePolicy;
    public ModelAsset LWalkPolicy;
    public ModelAsset LJumpPolicy;
    StyleB LastBmotion;
    StyleQ LastQmotion;
    StyleL LastLmotion;
    GameObject robot;
    public bool accelerate;
    public bool train;
    float[] kb = new float[23]{30,30,30,30,30,30,30,30,30,30, 30,30,30,30,30,30,30,30,30,30, 30,30,30};
    float[] kb1 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] kb2 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float ko = 0.5f;
    float kh = 0;
    //float dh = 25;
    //float d0 = 15;
    //float uf1 = 0;
    //float uf2 = 0;
    public bool fixbody = false;
    public int ObservationNum;
  
    public override void Initialize()
    {

        arts = this.GetComponentsInChildren<ArticulationBody>();
        art0 = arts[0];
        body = art0.transform;
        List<ArticulationBody> controllableJoints = new List<ArticulationBody>();
        List<ArticulationBody> actsList = new List<ArticulationBody>();

        for (int k = 0; k < arts.Length; k++) controllableJoints.Add(arts[k]);;
        jh = controllableJoints.ToArray();

        pos0 = body.position;
        rot0 = body.rotation;
        art0.GetJointPositions(P0);
        art0.GetJointVelocities(W0);

        LoadActionFolders("./Assets/Retarget/feedforward");
        print(actionFolders[0]);
        LoadDataForAction(actionFolders[0]);

        accelerate = train;

    }
    void LoadActionFolders(string basePath)
    {
        string[] folders = Directory.GetDirectories(basePath);
        actionFolders = folders.ToList();  
    }
    void LoadDataForAction(string actionFolder)
    {
        dofFilePath = Path.Combine(actionFolder, "dof.csv");
        rotFilePath = Path.Combine(actionFolder, "root_rot.csv");
        posFilePath = Path.Combine(actionFolder, "root_trans_offset.csv");

        allDofData = LoadDataFromFile(dofFilePath);
        allRotData = LoadDataFromFile(rotFilePath);
        allPosData = LoadDataFromFile(posFilePath);
    }

    List<float[]> LoadDataFromFile(string filePath)
    {
        List<float[]> dataList = new List<float[]>();
        try
        {
            string[] lines = File.ReadAllLines(filePath);
            foreach (string line in lines)
            {
                string[] values = line.Split(',');
                List<float> frameData = new List<float>();
                foreach (string value in values)
                {
                    if (float.TryParse(value.Trim(), out float parsedValue))
                    {
                        frameData.Add(parsedValue);
                    }
                }
                dataList.Add(frameData.ToArray());
            }
        }
        catch (System.Exception e)
        {
            print("Error loading data from file " + filePath + ": " + e.Message);
        }

        return dataList;
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
                clone.GetComponent<H1Agent>()._isClone = true; 
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
        art0.TeleportRoot(pos0, rot0);
        art0.velocity = Vector3.zero;
        art0.angularVelocity = Vector3.zero;
        print($"PO={P0.Count}");
        //art0.SetJointPositions(P0);
        print($"WO={W0.Count}");
        //art0.SetJointVelocities(W0);
        for (int i = 0; i < 19; i++) SetJointTargetDeg(jh[i], u0[i]);
        for (int i = 0; i < 19; i++) u[i] = 0;
        for (int i = 0; i < 19; i++) uff[i] = 0;
        ObservationNum = 40;
        LoadDataForAction(actionFolders[currentActionIndex]);
        currentFrame = 0;
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
        sensor.AddObservation(EulerTrans(body.eulerAngles[0]) * 3.14f / 180f );//rad
        sensor.AddObservation(EulerTrans(body.eulerAngles[2]) * 3.14f / 180f );//rad
        sensor.AddObservation(body.InverseTransformDirection(art0.velocity));
        sensor.AddObservation(body.InverseTransformDirection(art0.angularVelocity));
        for (int i = 0; i < jh.Length; i++)
        {
            ArticulationBody currentJoint = jh[i];

            // 获取该关节的自由度位置
            var jointPos = currentJoint.jointPosition;
            // 遍历该关节的所有自由度，并将它们作为观察值添加
            for (int j = 0; j < jointPos.dofCount; j++)
            {
                sensor.AddObservation(jointPos[j]);
            }

            // 获取该关节的自由度速度
            var jointVel = currentJoint.jointVelocity;
            // 遍历该关节的所有自由度，并将它们作为观察值添加
            for (int j = 0; j < jointVel.dofCount; j++)
            {
                sensor.AddObservation(jointVel[j]);
            }
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
        for (int i = 0; i < 19; i++) utotal[i] = 0;
        var continuousActions = actionBuffers.ContinuousActions;
        var kk = 0.9f;

        for (int i = 0; i < 19; i++)
        {
            u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
            u0[i] += u[i];
            uff[i] += u0[i];
            utotal[i] = kb[i] * u[i] + kb1[i] * u0[i] + kb2[i] * uff[i];
            if (fixbody) utotal[i] = 0;
        }
        string name = this.name;
        int[] idx = new int[6] { 2, 3, 4, 7, 8, 9 };
        if (name.Contains("MyRobot"))
        {
            idx = new int[6] { 2, 3, 4, 7, 8, 9 };//biped with 5 joints in each leg, change here if the feedforward is incorrect 
        }
        if (Robot == StyleR.biped) { }
        if (Robot == StyleR.legwheeled) { }
        if (Robot == StyleR.quadruped)
        {
            //dh = 15;
            //d0 = 18;
            if (QuadrupedTargetMotion == StyleQ.trot) { }
            if (QuadrupedTargetMotion == StyleQ.march) { }
            if (QuadrupedTargetMotion == StyleQ.wave)
            {
                ko = 0.5f;
                kh = 0;
                //dh = 20;
                //d0 = 0;
                //下面的方法是默认抄的，是修改的地方
                /*
                utotal[1] += dh * uf1 + d0;
                utotal[2] += (dh * uf1 + d0) * -2;
                utotal[4] += dh * uf1 + d0;
                utotal[5] += (dh * uf1 + d0) * -2;
                utotal[7] += dh * uf2 + d0;
                utotal[8] += (dh * uf2 + d0) * -2;
                utotal[10] += dh * uf2 + d0;
                utotal[11] += (dh * uf2 + d0) * -2;
                */
                if (!train) SetModel("Quadruped", QWavePolicy);
            }
        }
        for (int i = 0; i < 19; i++) SetJointTargetDeg(jh[i], utotal[i]);
        if (BipedTargetMotion != LastBmotion || QuadrupedTargetMotion != LastQmotion || LegwheelTargetMotion != LastLmotion) EndEpisode();
        LastBmotion = BipedTargetMotion;
        LastQmotion = QuadrupedTargetMotion;
        LastLmotion = LegwheelTargetMotion;
    }
    void FixedUpdate()
    {
        if (allDofData.Count > 0)
        {
            if (accelerate) Time.timeScale = 20;
            if (!accelerate) Time.timeScale = 1;
            float[] currentDof = allDofData[currentFrame];
            float[] currentPos = allPosData[currentFrame];
            float[] currentRot = allRotData[currentFrame];
            for (int i = 0; i < 19; i++)
            {
                uff[i] = currentDof[i] * 180f / 3.14f;
                SetJointTargetDeg(jh[i], uff[i]);
            }

            Vector3 newPosition = new Vector3(-currentPos[1], currentPos[2], currentPos[0]);
            Quaternion newRotation = new Quaternion(
                currentRot[1],
                -currentRot[2],
                currentRot[0],
                currentRot[3]
            );

            art0.TeleportRoot(newPosition, newRotation);
            art0.velocity = Vector3.zero;
            art0.angularVelocity = Vector3.zero;

            currentFrame = (currentFrame + 1) % allDofData.Count;

            if (currentFrame == 0)
            {
                currentActionIndex = (currentActionIndex + 1) % actionFolders.Count;
                print("Switched to action: " + actionFolders[currentActionIndex]);
                EndEpisode();
                currentFrame = 0;
            }

            if (isEndEpisode)
            {
                currentFrame = 0;
                isEndEpisode = false;
            }
        }
        var vel = body.InverseTransformDirection(arts[0].velocity);
        var wel = body.InverseTransformDirection(arts[0].angularVelocity);
        var live_reward = 1f;
        var ori_reward1 = -0.1f * Mathf.Abs(EulerTrans(body.eulerAngles[0]));
        var ori_reward2 = -2f * Mathf.Abs(wel[1]);
        var ori_reward3 = -0.1f * Mathf.Abs(EulerTrans(body.eulerAngles[2]));
        var vel_reward1 = vel[2] - Mathf.Abs(vel[0]);
        var vel_reward2 = vel[2] - Mathf.Abs(vel[0]) + kh * Mathf.Abs(vel[1]);
        var reward = live_reward + (ori_reward1 + ori_reward2 + ori_reward3) * ko + vel_reward2;


        //var reward = 0;
        
        AddReward(reward);
        if (Mathf.Abs(EulerTrans(body.eulerAngles[0])) > 20f || Mathf.Abs(EulerTrans(body.eulerAngles[2])) > 20f)
        {
            if(train)EndEpisode();
        }
    }


    void SetJointTargetDeg(ArticulationBody joint, float x)
    {
        var drive = joint.xDrive;
        drive.stiffness = 2000f;//180f;
        drive.damping = 200f;//8f;
        drive.forceLimit = 300f;//250f;// 33.5f;// 50f;
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

    float ang_trans(ArticulationBody joint, float x)
    {
        x = (x + 1f) * 0.5f;
        var ang = Mathf.Lerp(joint.xDrive.lowerLimit, joint.xDrive.upperLimit, x);
        return ang;
    }
    float rad_normalize(ArticulationBody joint, float ang)
    {
        var ang1 = ang * 180f / 3.14f;
        var x = (ang1 - joint.xDrive.lowerLimit) / (joint.xDrive.upperLimit - joint.xDrive.lowerLimit);//0~1
        x = 2 * x - 1;//-1~1
        return x;
    }
    

}
