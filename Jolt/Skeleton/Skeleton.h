// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/Reference.h>
#include <Jolt/Core/Result.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/ObjectStream/SerializableObject.h>

JPH_NAMESPACE_BEGIN

#ifdef JPH_OBJECT_STREAM
class StreamIn;
class StreamOut;
#endif

/// Resource that contains the joint hierarchy for a skeleton
class JPH_EXPORT Skeleton : public RefTarget<Skeleton>
{
#ifdef JPH_OBJECT_STREAM
	JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Skeleton)
#endif

public:
	using SkeletonResult = Result<Ref<Skeleton>>;

	/// Declare internal structure for a joint
	class Joint
	{
#ifdef JPH_OBJECT_STREAM
		JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Joint)
#endif

	public:
		Joint() = default;
		Joint(
			const string_view &inName,
#ifdef JPH_OBJECT_STREAM
			const string_view &inParentName,
#endif
			int inParentJointIndex)
			: mName(inName),
#ifdef JPH_OBJECT_STREAM
			  mParentName(inParentName),
#endif
			  mParentJointIndex(inParentJointIndex)
		{
		}

		String mName; ///< Name of the joint
#ifdef JPH_OBJECT_STREAM
		String mParentName; ///< Name of parent joint
#endif
		int mParentJointIndex = -1; ///< Index of parent joint (in mJoints) or -1 if it has no parent
	};

	using JointVector = Array<Joint>;
	using JointNameMap = UnorderedMap<string_view, int>;

	///@name Access to the joints
	///@{
	const JointVector &		GetJoints() const															{ return mJoints; }
	JointVector &			GetJoints()																	{ return mJoints; }
	int						GetJointCount() const														{ return (int)mJoints.size(); }
	const Joint &			GetJoint(int inJoint) const													{ return mJoints[inJoint]; }
	Joint &					GetJoint(int inJoint)														{ return mJoints[inJoint]; }
#ifdef JPH_OBJECT_STREAM
	int						AddJoint(const string_view &inName, const string_view &inParentName = string_view());
#endif
	int						AddJoint(const string_view &inName, int inParentIndex);
	///@}

	/// Find joint by name
	int						GetJointIndex(const string_view &inName) const;

#ifdef JPH_OBJECT_STREAM
	/// Fill in parent joint indices based on name
	void					CalculateParentJointIndices();
#endif

#ifdef JPH_ENABLE_ASSERTS
	/// Many of the algorithms that use the Skeleton class require that parent joints are in the mJoints array before their children.
	/// This function returns true if this is the case, false if not.
	bool					AreJointsCorrectlyOrdered() const;
#endif

#ifdef JPH_OBJECT_STREAM
	/// Saves the state of this object in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restore the state of this object from inStream.
	static SkeletonResult	sRestoreFromBinaryState(StreamIn &inStream);
#endif

private:
	/// Joints
	JointVector				mJoints;
	/// Joint Names
	JointNameMap            mJointNameMap;
};

JPH_NAMESPACE_END
