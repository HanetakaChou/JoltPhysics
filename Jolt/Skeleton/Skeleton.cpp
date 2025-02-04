// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>

JPH_NAMESPACE_BEGIN

#ifdef JPH_OBJECT_STREAM
JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(Skeleton::Joint)
{
	JPH_ADD_ATTRIBUTE(Joint, mName)
	JPH_ADD_ATTRIBUTE(Joint, mParentName)
}

JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(Skeleton)
{
	JPH_ADD_ATTRIBUTE(Skeleton, mJoints)
}
#endif

#ifdef JPH_OBJECT_STREAM
int Skeleton::AddJoint(const string_view &inName, const string_view &inParentName)
{
	mJoints.emplace_back(inName, inParentName, -1);

	int i = (int)mJoints.size() - 1;

	JointNameMap::const_iterator j = mJointNameMap.find(mJoints[i].mName);
	JPH_ASSERT(mJointNameMap.end() == j);
	mJointNameMap.emplace_hint(mJointNameMap.end(), mJoints[i].mName, i);

	return i;
}
#endif

int Skeleton::AddJoint(const string_view &inName, int inParentIndex)
{
	mJoints.emplace_back(
		inName,
#ifdef JPH_OBJECT_STREAM
		inParentIndex >= 0 ? mJoints[inParentIndex].mName : String(),
#endif
		inParentIndex);

	int i = (int)mJoints.size() - 1;

	JointNameMap::const_iterator j = mJointNameMap.find(mJoints[i].mName);
	JPH_ASSERT(mJointNameMap.end() == j);
	mJointNameMap.emplace_hint(j, mJoints[i].mName, i);

	return i;
}

int Skeleton::GetJointIndex(const string_view &inName) const
{
	JointNameMap::const_iterator j = mJointNameMap.find(inName);

	if (mJointNameMap.end() != j)
	{
		return j->second;
	}
	else
	{
		return -1;
	}
}

#ifdef JPH_OBJECT_STREAM
void Skeleton::CalculateParentJointIndices()
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
	{
		mJointNameMap.emplace_hint(mJointNameMap.end(), mJoints[i].mName, i);
	}

	for (int i = 0; i < (int)mJoints.size(); ++i)
	{
		mJoints[i].mParentJointIndex = GetJointIndex(mJoints[i].mParentName);
	}
}
#endif

#ifdef JPH_ENABLE_ASSERTS
bool Skeleton::AreJointsCorrectlyOrdered() const
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
	{
		if (mJoints[i].mParentJointIndex >= i)
		{
			return false;
		}
	}

	return true;
}
#endif


#ifdef JPH_OBJECT_STREAM
void Skeleton::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write((uint32)mJoints.size());
	for (const Joint &j : mJoints)
	{
		inStream.Write(j.mName);
		inStream.Write(j.mParentJointIndex);
		inStream.Write(j.mParentName);
	}
}

Skeleton::SkeletonResult Skeleton::sRestoreFromBinaryState(StreamIn &inStream)
{
	Ref<Skeleton> skeleton = new Skeleton;

	uint32 len = 0;
	inStream.Read(len);
	skeleton->mJoints.resize(len);
	for (Joint &j : skeleton->mJoints)
	{
		inStream.Read(j.mName);
		inStream.Read(j.mParentJointIndex);
		inStream.Read(j.mParentName);
	}

	SkeletonResult result;
	if (inStream.IsEOF() || inStream.IsFailed())
		result.SetError("Failed to read skeleton from stream");
	else
		result.Set(skeleton);
	return result;
}
#endif

JPH_NAMESPACE_END
