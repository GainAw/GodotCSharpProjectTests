using Godot;
using System;
using static Godot.GD;

[Tool]
public class TwoBone3DSolver : Node
{
	//Target NodePaths
	[Export] private NodePath _targetNodePath;
	[Export] private NodePath _targetSkeletonNodePath;

	//String name of the tip bone used for the chain
	[Export] private string _tipBoneName;

	//Magnet position in respect to the Target Skeleton
	[Export] public Vector3 magnetVector;	

	//Bool to use the provided Magnet or not
	[Export] private bool _useMagnet = false;

	//References to the target Nodes
	private Skeleton _targetSkeleton;
	private Spatial _targetNode;

	//The Ids of the bone chain
	private int[] _boneIds = new int[3];

	//Base transforms of the bones in the bone chain
	private Transform[] _boneBasePose = new Transform[3];

	//Individual lengths of the bones
	private float[] _boneLengths = new float[2];

	//Floats to tell whether the target is out of the bones range either under or over
	private float _maxChainLength;
	private float _minChainLength;

	//Bool if the ik system is running or not
	[Export] private bool _isRunning = false;

	public override void _Ready()
	{
		//Sets reference to the Node used at ready
		_targetSkeleton = GetNode<Skeleton>(_targetSkeletonNodePath);
		_targetNode = GetNode<Spatial>(_targetNodePath);

		//Gets the bone Ids of the skeleton
		_boneIds[2] = _targetSkeleton.FindBone(_tipBoneName);
		_boneIds[1] = _targetSkeleton.GetBoneParent(_boneIds[2]);
		_boneIds[0] = _targetSkeleton.GetBoneParent(_boneIds[1]);

		//Calculates the lengths of the bones
		_boneLengths[0] = _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin.DistanceTo(_targetSkeleton.GetBoneGlobalPose(_boneIds[1]).origin);
		_boneLengths[1] = _targetSkeleton.GetBoneGlobalPose(_boneIds[1]).origin.DistanceTo(_targetSkeleton.GetBoneGlobalPose(_boneIds[2]).origin);

		//Calculates the Out of range variables
		_maxChainLength = _boneLengths[0];
		_maxChainLength += _boneLengths[1];

		_maxChainLength *= _maxChainLength;

		_minChainLength = Mathf.Abs(_boneLengths[0] - _boneLengths[1]);

		_minChainLength *= _minChainLength;

		//Sets the base transforms to what the skeleton starts them as
		_boneBasePose[0] = _targetSkeleton.GetBonePose(_boneIds[0]);
		_boneBasePose[1] = _targetSkeleton.GetBonePose(_boneIds[1]);
		_boneBasePose[2] = _targetSkeleton.GetBonePose(_boneIds[2]);
	}

	//Starts the Solver
	public void Start()
	{
		_isRunning = true;
	}

	//Stops the Solver
	public void Stop()
	{
		_isRunning = false;
	}

	public override void _Process(float delta)
	{
		solveIk();
	}

	//Solves for the ik chain
	private void solveIk()
	{
		if (_isRunning)
		{
			//Makes the Targetnode play nice with the skeleton by making it's values local
			Transform targetTransform = new Transform(new Basis(_targetNode.GlobalTransform.basis.Quat() * _targetSkeleton.GlobalTransform.basis.Quat()), _targetSkeleton.ToLocal(_targetNode.GlobalTransform.origin));

			//Resets the bones to the base bones in case there is a bug so that the solver doesn't stay broken
			_targetSkeleton.SetBonePose(_boneIds[0], _boneBasePose[0]);
			_targetSkeleton.SetBonePose(_boneIds[1], _boneBasePose[1]);
			_targetSkeleton.SetBonePose(_boneIds[2], _boneBasePose[2]);

			//Float to get the distance to the target from the root bone Global pose origin
			float targetDistanceSquared = targetTransform.origin.DistanceSquaredTo(LocalPoseToGlobalPose(_boneIds[0], _boneBasePose[0]).origin);

			//Checks whether the bone is in range or not
			if (targetDistanceSquared >= _maxChainLength || targetDistanceSquared <= _minChainLength) //Out of range Solver
			{
				for (int i = 0; i < 2; i++)
				{
					//Uses Normal math to point the chain at the target if it is out of range
					Vector3 boneNormal = _targetSkeleton.GetBoneGlobalPose(_boneIds[i + 1]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[i]).origin;
					Vector3 targetNormal = targetTransform.origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[i]).origin;
					Transform boneTransform = _targetSkeleton.GetBoneGlobalPose(_boneIds[i]);
					boneTransform.basis = boneTransform.basis.Rotated(boneNormal.Cross(targetNormal).Normalized(), boneNormal.AngleTo(targetNormal));

					_targetSkeleton.SetBonePose(_boneIds[i], GlobalPoseToLocalPose(_boneIds[i], boneTransform));
				}
			}
			else //Solve for Two Bone
			{
				//Makes a target vector based on the target and the root bone
				Vector3 targetVector = targetTransform.origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin;				

				//Variable to hold lengths needed for law of cosigns
				float[,] lengths = new float[2, 3];
				//Target Triangle lengths
				lengths[0, 0] = _boneLengths[0];
				lengths[0, 1] = _boneLengths[1];
				lengths[0, 2] = targetVector.Length();

				//Current Triangle Lengths
				lengths[1, 0] = _boneLengths[0];
				lengths[1, 1] = _boneLengths[1];
				lengths[1, 2] = (_targetSkeleton.GetBoneGlobalPose(_boneIds[2]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin).Length();

				//Get Bone Vectors
				Vector3[] boneVector = new Vector3[2];
				boneVector[0] = _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[1]).origin;
				boneVector[1] = _targetSkeleton.GetBoneGlobalPose(_boneIds[2]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[1]).origin;

				//Get angles need for rotation
				float currentBoneAngle = LawOfCosigns(lengths[1, 2], lengths[1, 1], lengths[1, 0]);
				float targetBoneAngle = LawOfCosigns(lengths[0 ,2], lengths[0, 1], lengths[0, 0]);

				//Solve for the inner angle of the elbow bone by subtracting the targetAngle by the elbow's current inner angle
				float angleToRotate = targetBoneAngle - currentBoneAngle;

				//Get elbow axis of upper arm and lower arm
				Vector3 elbowAxis = (boneVector[0].Cross(boneVector[1])).Normalized();

				//Set elbow inner angle of the second bone transform with the elbowAxis
				Transform boneTransform = _targetSkeleton.GetBoneGlobalPose(_boneIds[1]);
				boneTransform.basis = boneTransform.basis.Rotated(elbowAxis, angleToRotate);

				//Set Pose of the elbow with the new transform
				_targetSkeleton.SetBonePose(_boneIds[1], GlobalPoseToLocalPose(_boneIds[1], boneTransform));

				//settings up total chain vector (Vector from the root bone to the tip bone)
				Vector3 chainVector = (_targetSkeleton.GetBoneGlobalPose(_boneIds[2]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin);

				//Get the root bone transform and rotate so that it aligns with the target Vector
				boneTransform = _targetSkeleton.GetBoneGlobalPose(_boneIds[0]);
				boneTransform.basis = boneTransform.basis.Rotated(chainVector.Cross(targetVector).Normalized(), chainVector.AngleTo(targetVector));

				//Set Shoulder rotation
				_targetSkeleton.SetBonePose(_boneIds[0], GlobalPoseToLocalPose(_boneIds[0], boneTransform));

				//Solve for magnet
				if (_useMagnet)
				{
					//Find the arm chain normal and use it for the normal of the plane
					chainVector = (_targetSkeleton.GetBoneGlobalPose(_boneIds[2]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin);
					Vector3 chainNormal = chainVector.Normalized();
					Plane magnetPlane = new Plane(chainNormal, 0);

					//Project both the magnet and the elbow to said plane and return their positions;
					Vector3 elbowProject = magnetPlane.Project(_targetSkeleton.GetBoneGlobalPose(_boneIds[1]).origin - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin);
					Vector3 magnetProject = magnetPlane.Project(magnetVector - _targetSkeleton.GetBoneGlobalPose(_boneIds[0]).origin);

					//Find the signed rotation between the positions
					float rotation = SignedAngle(elbowProject, magnetProject, chainNormal);

					//Rotate and apply bone rotations
					boneTransform = _targetSkeleton.GetBoneGlobalPose(_boneIds[0]);
					boneTransform.basis = boneTransform.basis.Rotated(chainNormal, rotation);
					_targetSkeleton.SetBonePose(_boneIds[0], GlobalPoseToLocalPose(_boneIds[0], boneTransform));
				}

			}
			//SetEnd to be target rotation
			Transform endBoneTransform = _targetSkeleton.GetBoneGlobalPose(_boneIds[2]);
			endBoneTransform.basis = new Basis(targetTransform.basis.Quat());
			_targetSkeleton.SetBonePose(_boneIds[2], GlobalPoseToLocalPose(_boneIds[2], endBoneTransform));
		}
	}

	//Helper Functions

	//Law of cosigns function that returns the angle of a given triangle given the sides (will solve differently depending on what side is in what variable)
	private float LawOfCosigns(float a, float b, float c)
	{
		return Mathf.Acos(((b * b) + (c * c) - (a * a)) / (2 * b * c));
	}

	/*
	Unity function SignedAngle to help with magnet
	Takes the smallest angle between the vectors and finds which direction the rotaion goes
	*/
	public float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
	{
		float unsignedAngle = from.AngleTo(to);

		float cross_X = from.y * to.z - from.z * to.y;
		float cross_Y = from.z * to.x - from.x * to.z;
		float cross_Z = from.x * to.y - from.y * to.x;

		float sign = Math.Sign(axis.x * cross_X + axis.y * cross_Y + axis.z * cross_Z);
		return unsignedAngle * sign;
	}

	/*
	Translates the Global pose to a Local pose of a given bone id and returns local pose transform
	Given by TwistedTwigLeg on forum post https://godotforums.org/discussion/23291/dealing-with-bones-in-godot
	*/
	public Transform GlobalPoseToLocalPose(int boneId, Transform boneGlobalPose)
	{
		int boneParent = _targetSkeleton.GetBoneParent(boneId);
		if (boneParent >= 0)
		{
			Transform conversionTransform = _targetSkeleton.GetBoneGlobalPose(boneParent) * _targetSkeleton.GetBoneRest(boneId) * _targetSkeleton.GetBoneCustomPose(boneId);
			conversionTransform = conversionTransform.AffineInverse() * boneGlobalPose;

			//fix inaccuraces
			Vector3 originFix = conversionTransform.origin;
			originFix.x = Mathf.IsZeroApprox(originFix.x) ? 0 : originFix.x;
			originFix.y = Mathf.IsZeroApprox(originFix.y) ? 0 : originFix.y;
			originFix.z = Mathf.IsZeroApprox(originFix.z) ? 0 : originFix.z;

			Basis basisFix = conversionTransform.basis;
			for (int g = 0; g < 3; g++)
			{
				Vector3 fixer = basisFix[g];
				for (int i = 0; i < 3; i++)
				{
					fixer[i] = Mathf.IsZeroApprox(fixer[i]) ? 0 : fixer[i];
				}
				basisFix[g] = fixer;
			}
			conversionTransform = new Transform(basisFix, originFix);

			return conversionTransform;
		}
		else
		{
			return boneGlobalPose;
		}
	}

	/*
	Translates the Local pose to a Global pose of a given bone id and returns global pose transform
	Given by TwistedTwigLeg on forum post https://godotforums.org/discussion/23291/dealing-with-bones-in-godot
	*/
	public Transform LocalPoseToGlobalPose(int boneId, Transform boneLocalPose)
	{
		int boneParent = _targetSkeleton.GetBoneParent(boneId);
		if (boneParent >= 0)
		{
			Transform conversionTransform = _targetSkeleton.GetBoneGlobalPose(boneParent) * _targetSkeleton.GetBoneRest(boneId) * _targetSkeleton.GetBoneCustomPose(boneId) * _targetSkeleton.GetBonePose(boneId);
			return conversionTransform;
		}
		else
		{
			return boneLocalPose;
		}
	}
}
