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
        		
        //private Transform baseFootprint;
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

            //this.trajectoryKeyList = new List<string>(trajectoryInfoMap.Keys);
            //this.baseFootprint = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintName);
            this.baseFootprintPosNoise = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintPosNoiseName);
            this.baseFootprintRotNoise = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintRotNoiseName);
            this.baseFootprintRigidbody = SIGVerseUtils.FindTransformFromChild(this.transform.root, HSRCommon.BaseFootPrintRigidbodyName);

            this.initialPosition = this.baseFootprintRigidbody.position;//save initial position.
            this.initialRotation = this.baseFootprintRigidbody.rotation;

            Debug.Log(initialPosition);
            Debug.Log(initialRotation);
        }


		protected override void Start()
		{
			base.Start();
		}

		protected override void SubscribeMessageCallback(SIGVerse.RosBridge.trajectory_msgs.JointTrajectory jointTrajectory)
		{

            Debug.Log("omni callback");

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
		}

		protected override void Update()
		{
			base.Update();

            if (this.trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] != null || this.trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] != null || this.trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] != null)
            {
                Vector3 NewPosition = new Vector3();
                Vector3 NewNoisePos = new Vector3();
                this.GetOmniXY_NewPosition(this.trajectoryInfoMap, ref NewPosition, ref NewNoisePos);
                this.baseFootprintRigidbody.position = NewPosition;
                this.baseFootprintPosNoise.position = NewNoisePos;

                Debug.Log(NewPosition);

                Quaternion newRotation = new Quaternion();
                Quaternion newNoiseRot = new Quaternion();
                this.GetOmniT_NewRotation(this.trajectoryInfoMap, ref newRotation, ref newNoiseRot);
                this.baseFootprintRigidbody.rotation = newRotation;
                this.baseFootprintRotNoise.rotation = newNoiseRot;

            }

        }//Update


        private void GetOmniXY_NewPosition(Dictionary<string, TrajectoryInfo> trajectoryInfoMap, ref Vector3 Position, ref Vector3 NoisePos)
        {
            TrajectoryInfo trajectoryInfo = trajectoryInfoMap[HSRCommon.OmniOdomX_JointName];

            int targetPointIndex = 0;
            // Select current trajectory target point 
            for (int i = 0; i < trajectoryInfo.Durations.Count; i++)
            {
                targetPointIndex = i;
                if (Time.time - trajectoryInfo.StartTime < trajectoryInfo.Durations[targetPointIndex]){ break; }
            }

            float delta_time;//時間進捗を0-1の時間にマッピング
            if (targetPointIndex == 0)
            {
                delta_time = (Time.time - trajectoryInfo.StartTime) / (trajectoryInfo.Durations[targetPointIndex]);
            }
            else
            {
                delta_time = (Time.time - (trajectoryInfo.StartTime + trajectoryInfo.Durations[targetPointIndex - 1])) / (trajectoryInfo.Durations[targetPointIndex] - trajectoryInfo.Durations[targetPointIndex - 1]);
                this.startPosition = new Vector3(trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].GoalPositions[targetPointIndex - 1] + this.initialPosition.x, 0, trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions[targetPointIndex - 1] + this.initialPosition.z);
            }


            Vector3 goalPosition = new Vector3();//temp xy goal point
            goalPosition.x = trajectoryInfoMap[HSRCommon.OmniOdomY_JointName].GoalPositions[targetPointIndex] + this.initialPosition.x;
            goalPosition.z = trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions[targetPointIndex] + this.initialPosition.z;

            if (delta_time > 1)
            {
                Position = goalPosition;//calc new position.
                NoisePos = goalPosition;//calc new position.
                if ((targetPointIndex + 1) == trajectoryInfoMap[HSRCommon.OmniOdomX_JointName].GoalPositions.Count)
                {
                    trajectoryInfoMap[HSRCommon.OmniOdomX_JointName] = null;
                    trajectoryInfoMap[HSRCommon.OmniOdomY_JointName] = null;
                }
                return;
            }

            Position = Vector3.Slerp(this.startPosition, goalPosition, delta_time);//calc new position.
            NoisePos = Vector3.Slerp(this.startPosition, goalPosition, delta_time);//calc new position.

            return;
        }//GetOmniXY_NewPosition


        private void GetOmniT_NewRotation(Dictionary<string, TrajectoryInfo> trajectoryInfoMap, ref Quaternion Rotation, ref Quaternion NoiseRot)
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
                Rotation = goalRotation;//calc new rotation.
                NoiseRot = goalRotation;//calc new rotation.
                if ((targetPointIndex + 1) == trajectoryInfoMap[HSRCommon.OmniOdomT_JointName].GoalPositions.Count)
                {
                    trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] = null;
                }
                return;
            }

            Rotation = Quaternion.Slerp(this.startRotation, goalRotation, delta_time);//calc new rotation.
            NoiseRot = Quaternion.Slerp(this.startRotation, goalRotation, delta_time);//calc new rotation.
            
            //trajectoryInfoMap[HSRCommon.OmniOdomT_JointName] = null;
            trajectoryInfo.CurrentTime = Time.time;
            trajectoryInfo.CurrentPosition = Rotation.eulerAngles.z;
            
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

