//最后一次修改:2025/6/9 14：01 //其他部分基本完毕，导入前馈被修正，训练时，网络输出一直是0，说明训练逻辑仍有问题
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
    public bool fixbody = false;
    public bool train;
    public bool accelerate; 
    float[] uff = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] u = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] u0 = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] utt = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float[] utotal = new float[19] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 0, 0};
    float ko = 0.5f;
    float kh = 0;
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
    Transform body;
    public int ObservationNum;
    public int ActionNum;
    [Header("RobotType")]
    public StyleR Robot;
    [Header("Biped")]
    public StyleB BipedTargetMotion;
    public ModelAsset BWalkPolicy;
    public ModelAsset BRunPolicy;
    public ModelAsset BJumpPolicy; 
    [Header("quadruped")]
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
    List<float> P0 = new List<float>();
    List<float> W0 = new List<float>();
    //List<float> subList = new List<float>();
    List<Transform> bodypart = new List<Transform>();
    Vector3 pos0;
    Quaternion rot0;
    ArticulationBody[] arts = new ArticulationBody[40];
    ArticulationBody[] jh = new ArticulationBody[19];
    GameObject robot;
    float[] kb = new float[23]{30,30,30,30,30,30,30,30,30,30, 30,30,30,30,30,30,30,30,30,30, 30,30,30};
    float[] kb1 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    float[] kb2 = new float[23]{0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0};
    public override void Initialize()
    {

        arts = this.GetComponentsInChildren<ArticulationBody>();
        ActionNum = 0;
        print($"arts.Length = {arts.Length}");

        art0 = arts[0];
        //body = art0.transform;

        for (int k = 0; k < arts.Length; k++)
        {
            if (arts[k].jointType.ToString() == "RevoluteJoint")
            {
                jh[ActionNum] = arts[k];
                print($"jh[{ActionNum}] = {jh[ActionNum]}");
                ActionNum++;
            }
        }
        print($"ActionNum = {ActionNum}");

        body = art0.GetComponent<Transform>();
        pos0 = body.position;
        rot0 = body.rotation;
        print($"P0.Length = {P0.Count},initialize"); //0
        print($"W0.Length = {W0.Count},initialize"); //0
        art0.GetJointPositions(P0);
        art0.GetJointVelocities(W0);
        print($"P0.Length = {P0.Count},initialized"); //25
        print($"W0.Length = {W0.Count},initialized"); //25
        accelerate = train;

        LoadActionFolders("./Assets/Retarget/feedforward");
        print($"actionFolders[1]={actionFolders[0]}");
        LoadDataForAction(actionFolders[0]);
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
            for (int i = 1; i < 3; i++)
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

        print($"P0.Length = {P0.Count},onepisodebegin"); //25
        print($"W0.Length = {W0.Count},onepisodebegin"); //25
        art0.SetJointPositions(P0);
        art0.SetJointVelocities(W0);

        for (int i = 0; i < 19; i++) SetJointTargetDeg(jh[i], u0[i]);
        for (int i = 0; i < 19; i++) u[i] = 0;
        for (int i = 0; i < 19; i++) uff[i] = 0;

        ObservationNum = 9 + 2 * ActionNum;

        LoadDataForAction(actionFolders[currentActionIndex]);
        currentFrame = 0;

        if (fixbody) arts[0].immovable = false;
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
        //OR加了吗,好像还没加
        sensor.AddObservation(EulerTrans(body.eulerAngles[0]) * 3.14f / 180f );//rad
        sensor.AddObservation(EulerTrans(body.eulerAngles[2]) * 3.14f / 180f );//rad
        sensor.AddObservation(body.InverseTransformDirection(art0.velocity));
        sensor.AddObservation(body.InverseTransformDirection(art0.angularVelocity));
        for (int i = 0; i < jh.Length ; i++)
        {
            sensor.AddObservation((jh[i].jointPosition[0]));
            sensor.AddObservation(jh[i].jointVelocity[0]);
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
        for (int i = 0; i < jh.Length; i++) utotal[i] = 0;
        var continuousActions = actionBuffers.ContinuousActions;
        var kk = 0.9f;

        for (int i = 0; i < jh.Length; i++)
        {
            //print($"u[{i}] = {u[i]} , continuousActions[{i}]={continuousActions[i]}");
            u[i] = u[i] * kk + (1 - kk) * continuousActions[i];
            print($"continuousActions[{i}] = {continuousActions[i]}");
        }
        string name = this.name;
        if (Robot == StyleR.biped) { print("Robot == StyleR.biped"); }
        if (Robot == StyleR.legwheeled) { print("Robot == StyleR.legwheeled"); }
        if (Robot == StyleR.quadruped)
        {
            if (QuadrupedTargetMotion == StyleQ.trot)
            {
                print("QuadrupedTargetMotion == StyleQ.trot");
            }
            if (QuadrupedTargetMotion == StyleQ.march)
            {
                print("QuadrupedTargetMotion == StyleQ.march");
            }
            if (QuadrupedTargetMotion == StyleQ.wave)
            {
                if (!train && !fixbody)
                {
                    print("loading your model");
                    SetModel("gewu", QWavePolicy);
                }
                else
                {
                    if (allDofData.Count > 0)
                    {
                        float[] currentDof = allDofData[currentFrame];
                        float[] currentPos = allPosData[currentFrame];
                        float[] currentRot = allRotData[currentFrame];
                        for (int i = 0; i < 19; i++)
                        {
                            uff[i] = currentDof[i] * 180f / 3.14f;
                            if (fixbody)
                            {
                                SetJointTargetDeg(jh[i], uff[i]);
                                print($"Now,fixbody:SetJointTargetDeg(jh[i], uff[i]):u[{i}]={u[i]};u0[{i}]={u0[i]};utt[{i}]={utt[i]};utotal[{i}]={utotal[i]};uff[{i}]={uff[i]}");
                            }
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
                        if (!fixbody && train)
                        {
                            for (int i = 0; i < jh.Length; i++)
                            {
                                u0[i] += u[i];//一次积分
                                utt[i] += u0[i];//二次积分
                                utotal[i] = kb[i] * u[i] + uff[i] + kb1[i] * u0[i] + kb2[i] * utotal[i]; //at = a_{ff} + k_b * a_{fb}
                                utotal[i] = uff[i];
                                SetJointTargetDeg(jh[i], utotal[i]);
                                print($"Now,!fixbody && train:SetJointTargetDeg(jh[i], utotal[i]):u[{i}]={u[i]};u0[{i}]={u0[i]};utt[{i}]={utt[i]};utotal[{i}]={utotal[i]};uff[{i}]={uff[i]}");
                            }
                        }
                    }

                    /*
                    for (int i = 0; i < 19; i++)
                    {
                        print($"u[{i}]={u[i]};u0[{i}]={u0[i]};utt[{i}]={utt[i]};utotal[{i}]={utotal[i]};");
                    }
                    */
                    /*
                    if (allDofData.Count > 0)
                    {
                        float[] currentDof = allDofData[currentFrame];
                        float[] currentPos = allPosData[currentFrame];
                        float[] currentRot = allRotData[currentFrame];
                        for (int i = 0; i < 19; i++)
                        {
                            //uff[i] = currentDof[i] * 180f / 3.14f;
                            uff[i] = 0;
                            print($"uff[{i}]={uff[i]}");
                        }
                    }
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
                    for (int i = 0; i < jh.Length; i++)
                    {
                        u0[i] += u[i];//一次积分
                        utt[i] += u0[i];//二次积分
                        //utotal[i] = kb[i] * u[i] + uff[i] + kb1[i] * u0[i] + kb2[i] * utotal[i]; //at = a_{ff} + k_b * a_{fb}
                        //utotal[i] = uff[i];
                        utotal[i] = 0;
                        print($"utotal[{i}]={utotal[i]}");
                    }
                    if (!train) SetModel("gewu", QWavePolicy);
                    */
                }

            }
        }
    }
        void FixedUpdate()
        {
            var vel = body.InverseTransformDirection(arts[0].velocity);
            var wel = body.InverseTransformDirection(arts[0].angularVelocity);
            var live_reward = 1f;
            var ori_reward1 = -0.1f * Mathf.Abs(EulerTrans(body.eulerAngles[0]));
            var ori_reward2 = -2f * Mathf.Abs(wel[1]);
            var ori_reward3 = -0.1f * Mathf.Abs(EulerTrans(body.eulerAngles[2]));
            var vel_reward1 = vel[2] - Mathf.Abs(vel[0]);
            var vel_reward2 = vel[2] - Mathf.Abs(vel[0]) + kh * Mathf.Abs(vel[1]);
            var reward = live_reward + (ori_reward1 + ori_reward2 + ori_reward3) * ko + vel_reward2;
            //print($"this turn reward = {reward}");
            AddReward(reward);

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
