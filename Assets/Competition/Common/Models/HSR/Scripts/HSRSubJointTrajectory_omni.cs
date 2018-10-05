using UnityEngine;
using System.Collections;
using SIGVerse.RosBridge;
using SIGVerse.Common;
using System.Collections.Generic;
using SIGVerse.ToyotaHSR;
using System;

namespace SIGVerse.ToyotaHSR
{
	public class HSRSubJointTrajectory_omni : RosSubMessage<SIGVerse.RosBridge.trajectory_msgs.JointTrajectory>
	{
		public class TrajectoryInfo
		{
			public float StartTime    { get; set; }
			public List<float> Durations { get; set; }
			public List<float> GoalPositions { get; set; }
			public float CurrentTime     { get; set; }
			public float CurrentPosition { get; set; }

			public TrajectoryInfo(float startTime, List<float> duration, List<float> goalPosition, float currentTime, float currentPosition)
			{
				this.StartTime       = startTime;
				this.Durations       = duration;
				this.GoalPositions   = goalPosition;
				this.CurrentTime     = currentTime;
				this.CurrentPosition = currentPosition;
			}
		}
        		
        private Transform baseFootprintPosNoise;
        private Transform baseFootprintRotNoise;
        private Transform baseFootprintRigidbody;

		private Dictionary<string, TrajectoryInfo> trajectoryInfoMap;
		private List<string> trajectoryKeyList;
        
        private Vector3 initialPosition = new Vector3();
        private Quaternion initialRotation = new Quaternion();
        private Vector3 startPosition = new Vector3();
        private Quaternion startRotation = new Quaternion();
        

        void Awake()
		{
			this.trajectoryInfoMap = new Dictionary<string, TrajectoryInfo>();
            this.trajectoryInfoMap.Add(HSRCommon.OmniOdomX_JointName, null);
            this.trajectoryInfoMap.Add(HSRCommon.OmniOdomY_JointName, null);
            this.trajectoryInfoMap.Add(HSRCommon.OmniOdomT_JointName, null);

            this.baseFootprintPosNoise = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintPosNoiseName);
            this.baseFootprintRotNoise = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintRotNoiseName);
            this.baseFootprintRigidbody = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintRigidbodyName);

            this.initialPosition = this.baseFootprintRigidbody.position;//save initial position.
            this.initialRotation = this.baseFootprintRigidbody.rotation;
        }//Awake


        protected override void Start()
		{
			base.Start();
		}

		protected override void SubscribeMessageCallback(SIGVerse.RosBridge.trajectory_msgs.JointTrajectory jointTrajectory)
		{
            this.startPosition = this.baseFootprintRigidbody.position;//save start position.
            this.startRotation = this.baseFootprintRigidbody.rotation;


            if (jointTrajectory.joint_names.Count != jointTrajectory.points[0].positions.Count)
			{
				SIGVerseLogger.Warn("joint_names.Count != points.positions.Count  topicName = "+this.topicName);
				return;
			}

            for (int i=0; i < jointTrajectory.joint_names.Count; i++)
			{
				string name    = jointTrajectory.joint_names[i];

                List<float> positions = new List<float>();
                List<float> durations = new List<float>();
                for (int pointIndex = 0; pointIndex < jointTrajectory.points.Count; pointIndex++)
                {
                    positions.Add(HSRCommon.GetClampedPosition((float)jointTrajectory.points[pointIndex].positions[i], name));
                    durations.Add((float)jointTrajectory.points[pointIndex].time_from_start.secs + (float)jointTrajectory.points[pointIndex].time_from_start.nsecs * 1.0e-9f);
                }

                if (name == HSRCommon.OmniOdomX_JointName)
                {
                    this.trajectoryInfoMap[name] = new TrajectoryInfo(Time.time, durations, positions, Time.time, this.baseFootprintRigidbody.position.z);
                }

                if (name == HSRCommon.OmniOdomY_JointName)
                {
                    this.trajectoryInfoMap[name] = new TrajectoryInfo(Time.time, durations, positions, Time.time, this.baseFootprintRigidbody.position.x);
                }

                if (name == HSRCommon.OmniOdomT_JointName)
                {
                    this.trajectoryInfoMap[name] = new TrajectoryInfo(Time.time, durations, positions, Time.time, 0.0f);
                }
            }

            if(trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] == null || trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] == null || trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] == null)
            {
                trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] = null;
                trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] = null;
                trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] = null;
                SIGVerseLogger.Warn("Omni trajectry args error.");
            }

            //check limit speed
            for (int i = 0; i < trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions.Count; i++)
            {
                double linear_speed = 0;
                if (i == 0)
                {
                    double temp_distance = Math.Sqrt(Math.Pow(((trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions[0] + this.initialPosition.z) - this.startPosition.z) * 100, 2) + Math.Pow(((trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].GoalPositions[0] + this.initialPosition.x) - this.startPosition.x) * 100, 2));
                    linear_speed = (temp_distance / 100) / trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].Durations[0];
                }
                else
                {
                    double temp_distance = Math.Sqrt(Math.Pow((trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions[i] - trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions[i-1]) * 100, 2) + Math.Pow((trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].GoalPositions[i] - trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].GoalPositions[i-1]) * 100, 2));
                    linear_speed = (temp_distance / 100) / (trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].Durations[i] - trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].Durations[i-1]);
                }

                double angular_speed = 0;
                if (i == 0)
                {
                    double temp_angle = Math.Abs((this.ToAngle(trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].GoalPositions[0]) + this.initialRotation.eulerAngles.y) - this.startRotation.eulerAngles.y);
                    if(temp_angle > Math.PI) { temp_angle -= Math.PI; }
                    angular_speed = temp_angle / trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].Durations[0];
                }
                else
                {
                    double temp_angle = Math.Abs(this.ToAngle(trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].GoalPositions[i]) - trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].GoalPositions[i-1]);
                    if (temp_angle > Math.PI) { temp_angle -= Math.PI; }
                    angular_speed = temp_angle / (trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].Durations[i] - trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].Durations[i-1]);
                }

                if (linear_speed > HSRCommon.MaxSpeedBase || angular_speed > this.ToAngle(HSRCommon.MaxSpeedBaseRad))//exceed limit 
                {
                    trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] = null;
                    trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] = null;
                    trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] = null;
                    SIGVerseLogger.Warn("Omni trajectry args error. (exceed limit)");
                    return;
                }
                
            }//for


        }//SubscribeMessageCallback


        protected override void Update()
		{
			base.Update();

            //Debug.Log(this.baseFootprintRigidbody.rotation.eulerAngles.y);

            if (this.trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] != null && this.trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] != null && this.trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] != null)
            {
                Vector3 NewPosition = new Vector3();
                Vector3 NewNoisePos = new Vector3();
                this.GetOmniXY_NewPosition(ref this.trajectoryInfoMap, ref NewPosition, ref NewNoisePos);
                
                this.baseFootprintRigidbody.position = NewPosition;
                this.baseFootprintPosNoise.position = NewNoisePos;

                Quaternion newRotation = new Quaternion();
                Quaternion newNoiseRot = new Quaternion();
                this.GetOmniT_NewRotation(ref this.trajectoryInfoMap, ref newRotation, ref newNoiseRot);
                this.baseFootprintRigidbody.rotation = newRotation;
                this.baseFootprintRotNoise.rotation = newNoiseRot;
            }

        }//Update


        private void GetOmniXY_NewPosition(ref Dictionary<string, TrajectoryInfo> trajectoryInfoMap, ref Vector3 Position, ref Vector3 NoisePos)
        {
            TrajectoryInfo trajectoryInfo_x = trajectoryInfoMap[HSRCommon.OmniOdomX_JointName];
            TrajectoryInfo trajectoryInfo_y = trajectoryInfoMap[HSRCommon.OmniOdomY_JointName];


            int targetPointIndex = 0;
            // Select current trajectory target point 
            for (int i = 0; i < trajectoryInfo_x.Durations.Count; i++)
            {
                targetPointIndex = i;
                if (Time.time - trajectoryInfo_x.StartTime < trajectoryInfo_x.Durations[targetPointIndex]){ break; }
            }

            float delta_time;//時間進捗を0-1の時間にマッピング
            if (targetPointIndex == 0)
            {
                delta_time = (Time.time - trajectoryInfo_x.StartTime) / (trajectoryInfo_x.Durations[targetPointIndex]);
            }
            else
            {
                delta_time = (Time.time - (trajectoryInfo_x.StartTime + trajectoryInfo_x.Durations[targetPointIndex - 1])) / (trajectoryInfo_x.Durations[targetPointIndex] - trajectoryInfo_x.Durations[targetPointIndex - 1]);
                this.startPosition = new Vector3(trajectoryInfo_y.GoalPositions[targetPointIndex - 1] + this.initialPosition.x, 0, trajectoryInfo_x.GoalPositions[targetPointIndex - 1] + this.initialPosition.z);
            }


            Vector3 goalPosition = new Vector3();//temp xy goal point
            goalPosition.x = trajectoryInfo_y.GoalPositions[targetPointIndex] + this.initialPosition.x;
            goalPosition.z = trajectoryInfo_x.GoalPositions[targetPointIndex] + this.initialPosition.z;

            if (delta_time > 1)
            {
                Position = goalPosition;
                NoisePos = goalPosition;
                if ((targetPointIndex + 1) == trajectoryInfo_x.GoalPositions.Count)
                {
                    trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] = null;
                    trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] = null;
                }
                return;
            }

            Position = Vector3.Slerp(this.startPosition, goalPosition, delta_time);//calc new position.
            NoisePos = Vector3.Slerp(this.startPosition, goalPosition, delta_time);//calc new position.
            trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].CurrentPosition = Position.z;
            trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].CurrentPosition = Position.x;
            trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].CurrentTime = Time.time;
            trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].CurrentTime = Time.time;

            return;
        }//GetOmniXY_NewPosition


        private void GetOmniT_NewRotation(ref Dictionary<string, TrajectoryInfo> trajectoryInfoMap, ref Quaternion Rotation, ref Quaternion NoiseRot)
        {
            TrajectoryInfo trajectoryInfo = trajectoryInfoMap[HSRCommon.OmniOdomT_JointName];

            int targetPointIndex = 0;
            // Select current trajectory target point 
            for (int i = 0; i < trajectoryInfo.Durations.Count; i++)
            {
                targetPointIndex = i;
                if (Time.time - trajectoryInfo.StartTime < trajectoryInfo.Durations[targetPointIndex]) { break; }
            }


            float delta_time;//時間進捗を0-1の時間にマッピング
            if (targetPointIndex == 0)
            {
                delta_time = (Time.time - trajectoryInfo.StartTime) / (trajectoryInfo.Durations[targetPointIndex]);
            }
            else
            {
                delta_time = (Time.time - (trajectoryInfo.StartTime + trajectoryInfo.Durations[targetPointIndex - 1])) / (trajectoryInfo.Durations[targetPointIndex] - trajectoryInfo.Durations[targetPointIndex - 1]);
                this.startRotation = Quaternion.Euler(new Vector3(this.initialRotation.eulerAngles.x, this.initialRotation.eulerAngles.y, this.ToAngle(trajectoryInfo.GoalPositions[targetPointIndex - 1]) + this.initialRotation.eulerAngles.z));
            }


            Quaternion goalRotation = new Quaternion();//temp rotation goal point
            goalRotation = Quaternion.Euler(new Vector3(this.initialRotation.eulerAngles.x, this.initialRotation.eulerAngles.y, this.ToAngle(trajectoryInfo.GoalPositions[targetPointIndex]) + this.initialRotation.eulerAngles.z));

            if (delta_time > 1)
            {
                Rotation = goalRotation;
                NoiseRot = goalRotation;
                if ((targetPointIndex + 1) == trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].GoalPositions.Count)
                {
                    trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] = null;
                }
                return;
            }

            Rotation = Quaternion.Slerp(this.startRotation, goalRotation, delta_time);//calc new rotation.
            NoiseRot = Quaternion.Slerp(this.startRotation, goalRotation, delta_time);//calc new rotation.

            trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].CurrentTime = Time.time;
            trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].CurrentPosition = Rotation.eulerAngles.z;
            
            return;
        }//GetOmniT_RotationAndUpdateTrajectory
        

        private float GetPosNoise(float val)
        {
            float randomNumber = SIGVerseUtils.GetRandomNumberFollowingNormalDistribution(0.6f); // sigma=0.6

            return val * Mathf.Clamp(randomNumber, -3.0f, +3.0f); // plus/minus 3.0 is sufficiently large.
        }


        private float GetRotNoise(float val)
        {
            float randomNumber = SIGVerseUtils.GetRandomNumberFollowingNormalDistribution(0.3f); // sigma=0.3

            return val * Mathf.Clamp(randomNumber, -3.0f, +3.0f); // plus/minus 3.0 is sufficiently large.
        }


        public float ToAngle(double radian)
        {
            return (float)(radian * 180 / Math.PI);
        }

    }
}

