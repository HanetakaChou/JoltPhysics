// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

// Modified by Hanataka Chou(Yuqiao Zhang)
// LGPL

#include <Jolt/Jolt.h>
#include <Jolt/Skeleton/SkeletonMapper.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Core/UnorderedSet.h>

JPH_SUPPRESS_WARNING_PUSH
JPH_SUPPRESS_WARNINGS
#pragma GCC diagnostic ignored "-Wreserved-macro-identifier"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include "../../source/brx_animation_ik_reaching.h"
#include "../../../McRT-Malloc/include/mcrt_unordered_map.h"
JPH_SUPPRESS_WARNING_POP

JPH_NAMESPACE_BEGIN

void SkeletonMapper::Initialize(const Skeleton *inSkeleton1, const Mat44 *inNeutralPose1, const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2, const bool inLockTranslations, const MapJoint2To1 &inMapJoint2To1)
{
	{
		mcrt_vector<mcrt_string> skeleton_a_joint_names(static_cast<size_t>(inSkeleton1->GetJointCount()));
		mcrt_vector<uint32_t> skeleton_a_joint_parent_indices(static_cast<size_t>(inSkeleton1->GetJointCount()));
		mcrt_vector<brx_motion_rigid_transform> skeleton_a_bind_pose_local_space(static_cast<size_t>(inSkeleton1->GetJointCount()));
		{
			for (int skeleton1_joint_index = 0; skeleton1_joint_index < inSkeleton1->GetJointCount(); ++skeleton1_joint_index)
			{
				skeleton_a_joint_names[skeleton1_joint_index] = inSkeleton1->GetJoint(skeleton1_joint_index).mName;
				skeleton_a_joint_parent_indices[skeleton1_joint_index] = inSkeleton1->GetJoint(skeleton1_joint_index).mParentJointIndex;
			}

			SkeletonPose skeleton_1_bind_pose;
			skeleton_1_bind_pose.SetSkeleton(inSkeleton1);
			for (int skeleton1_joint_index = 0; skeleton1_joint_index < inSkeleton1->GetJointCount(); ++skeleton1_joint_index)
			{
				skeleton_1_bind_pose.GetJointMatrix(skeleton1_joint_index) = inNeutralPose1[skeleton1_joint_index];
			}
			skeleton_1_bind_pose.CalculateJointStates();
			skeleton_1_bind_pose.CalculateJointMatrices();
			for (int skeleton1_joint_index = 0; skeleton1_joint_index < inSkeleton1->GetJointCount(); ++skeleton1_joint_index)
			{
				const_cast<Mat44 *>(inNeutralPose1)[skeleton1_joint_index] = skeleton_1_bind_pose.GetJointMatrix(skeleton1_joint_index);
			}

			for (int skeleton1_joint_index = 0; skeleton1_joint_index < inSkeleton1->GetJointCount(); ++skeleton1_joint_index)
			{
				skeleton_1_bind_pose.GetJoint(skeleton1_joint_index).mRotation.GetXYZW().StoreFloat4(reinterpret_cast<JPH::Float4 *>(&skeleton_a_bind_pose_local_space[skeleton1_joint_index].m_rotation[0]));
				skeleton_1_bind_pose.GetJoint(skeleton1_joint_index).mTranslation.StoreFloat3(reinterpret_cast<JPH::Float3 *>(&skeleton_a_bind_pose_local_space[skeleton1_joint_index].m_translation[0]));
			}
		}
		brx_animation_skeleton skeleton_a(std::move(skeleton_a_joint_names), std::move(skeleton_a_joint_parent_indices), std::move(skeleton_a_bind_pose_local_space));

		mcrt_vector<mcrt_string> skeleton_b_joint_names(static_cast<size_t>(inSkeleton2->GetJointCount()));
		mcrt_vector<uint32_t> skeleton_b_joint_parent_indices(static_cast<size_t>(inSkeleton2->GetJointCount()));
		mcrt_vector<brx_motion_rigid_transform> skeleton_b_bind_pose_local_space(static_cast<size_t>(inSkeleton2->GetJointCount()));
		{
			for (int skeleton2_joint_index = 0; skeleton2_joint_index < inSkeleton2->GetJointCount(); ++skeleton2_joint_index)
			{
				skeleton_b_joint_names[skeleton2_joint_index] = inSkeleton2->GetJoint(skeleton2_joint_index).mName;
				skeleton_b_joint_parent_indices[skeleton2_joint_index] = inSkeleton2->GetJoint(skeleton2_joint_index).mParentJointIndex;
			}

			SkeletonPose skeleton_2_bind_pose;
			skeleton_2_bind_pose.SetSkeleton(inSkeleton2);
			for (int skeleton2_joint_index = 0; skeleton2_joint_index < inSkeleton2->GetJointCount(); ++skeleton2_joint_index)
			{
				skeleton_2_bind_pose.GetJointMatrix(skeleton2_joint_index) = inNeutralPose2[skeleton2_joint_index];
			}
			skeleton_2_bind_pose.CalculateJointStates();
			skeleton_2_bind_pose.CalculateJointMatrices();
			for (int skeleton2_joint_index = 0; skeleton2_joint_index < inSkeleton2->GetJointCount(); ++skeleton2_joint_index)
			{
				const_cast<Mat44 *>(inNeutralPose2)[skeleton2_joint_index] = skeleton_2_bind_pose.GetJointMatrix(skeleton2_joint_index);
			}

			for (int skeleton2_joint_index = 0; skeleton2_joint_index < inSkeleton2->GetJointCount(); ++skeleton2_joint_index)
			{
				skeleton_2_bind_pose.GetJoint(skeleton2_joint_index).mRotation.GetXYZW().StoreFloat4(reinterpret_cast<JPH::Float4 *>(&skeleton_b_bind_pose_local_space[skeleton2_joint_index].m_rotation[0]));
				skeleton_2_bind_pose.GetJoint(skeleton2_joint_index).mTranslation.StoreFloat3(reinterpret_cast<JPH::Float3 *>(&skeleton_b_bind_pose_local_space[skeleton2_joint_index].m_translation[0]));
			}
		}
		brx_animation_skeleton skeleton_b(std::move(skeleton_b_joint_names), std::move(skeleton_b_joint_parent_indices), std::move(skeleton_b_bind_pose_local_space));

		mcrt_unordered_map<std::string_view, uint32_t> skeleton_a_joint_names_map;
		{
			for (uint32_t skeleton_a_joint_index = 0U; skeleton_a_joint_index < skeleton_a.get_joint_count(); ++skeleton_a_joint_index)
			{
				mcrt_unordered_map<std::string_view, uint32_t>::const_iterator found = skeleton_a_joint_names_map.find(skeleton_a.get_joint_name(skeleton_a_joint_index));
				JPH_ASSERT(skeleton_a_joint_names_map.end() == found);
				skeleton_a_joint_names_map.emplace_hint(found, skeleton_a.get_joint_name(skeleton_a_joint_index), skeleton_a_joint_index);
			}
		}

		mcrt_vector<brx_animation_skeleton_mapper_ragdoll_direct_mapping_data> &direct_mappings = this->m_ragdoll_mapping_data.access_direct_mappings();
		assert(direct_mappings.empty());
		{
			uint32_t const skeleton_a_joint_count = skeleton_a.get_joint_count();

			uint32_t const skeleton_b_joint_count = skeleton_b.get_joint_count();

			mcrt_vector<bool> mapped_b(static_cast<size_t>(skeleton_b_joint_count), false);

			mcrt_vector<DirectX::XMMATRIX> bind_pose_a_model_space(static_cast<size_t>(skeleton_a_joint_count));
			{
				for (uint32_t skeleton_a_joint_index = 0U; skeleton_a_joint_index < skeleton_a_joint_count; ++skeleton_a_joint_index)
				{
					brx_motion_rigid_transform skeleton_a_joint_transform_local_space = skeleton_a.get_bind_pose_joint_transform_local_space(skeleton_a_joint_index);

					DirectX::XMMATRIX bind_pose_a_local_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(reinterpret_cast<DirectX::XMFLOAT4 const *>(&skeleton_a_joint_transform_local_space.m_rotation[0]))), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 const *>(&skeleton_a_joint_transform_local_space.m_translation[0]))));

					uint32_t const skeleton_a_parent_joint_index = skeleton_a.get_joint_parent_index(skeleton_a_joint_index);
					if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_a_parent_joint_index)
					{
						assert(skeleton_a_parent_joint_index < skeleton_a_joint_index);
						bind_pose_a_model_space[skeleton_a_joint_index] = DirectX::XMMatrixMultiply(bind_pose_a_local_transform, bind_pose_a_model_space[skeleton_a_parent_joint_index]);
					}
					else
					{
						bind_pose_a_model_space[skeleton_a_joint_index] = bind_pose_a_local_transform;
					}
				}
			}

			mcrt_vector<DirectX::XMMATRIX> bind_pose_b_model_space(static_cast<size_t>(skeleton_b_joint_count));
			{
				for (uint32_t skeleton_b_joint_index = 0U; skeleton_b_joint_index < skeleton_b_joint_count; ++skeleton_b_joint_index)
				{
					brx_motion_rigid_transform skeleton_b_joint_transform_local_space = skeleton_b.get_bind_pose_joint_transform_local_space(skeleton_b_joint_index);

					DirectX::XMMATRIX bind_pose_b_local_transform = DirectX::XMMatrixMultiply(DirectX::XMMatrixRotationQuaternion(DirectX::XMLoadFloat4(reinterpret_cast<DirectX::XMFLOAT4 const *>(&skeleton_b_joint_transform_local_space.m_rotation[0]))), DirectX::XMMatrixTranslationFromVector(DirectX::XMLoadFloat3(reinterpret_cast<DirectX::XMFLOAT3 const *>(&skeleton_b_joint_transform_local_space.m_translation[0]))));

					uint32_t const skeleton_b_parent_joint_index = skeleton_b.get_joint_parent_index(skeleton_b_joint_index);
					if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_b_parent_joint_index)
					{
						assert(skeleton_b_parent_joint_index < skeleton_b_joint_index);
						bind_pose_b_model_space[skeleton_b_joint_index] = DirectX::XMMatrixMultiply(bind_pose_b_local_transform, bind_pose_b_model_space[skeleton_b_parent_joint_index]);
					}
					else
					{
						bind_pose_b_model_space[skeleton_b_joint_index] = bind_pose_b_local_transform;
					}
				}
			}

			for (uint32_t skeleton_b_joint_index = 0; skeleton_b_joint_index < skeleton_b_joint_count; ++skeleton_b_joint_index)
			{
				uint32_t skeleton_a_joint_index;
				{
					mcrt_unordered_map<std::string_view, uint32_t>::const_iterator found_skeleton_a_joint = skeleton_a_joint_names_map.find(skeleton_b.get_joint_name(skeleton_b_joint_index));
					if (skeleton_a_joint_names_map.end() != found_skeleton_a_joint)
					{
						skeleton_a_joint_index = found_skeleton_a_joint->second;
					}
					else
					{
						skeleton_a_joint_index = BRX_MOTION_UINT32_INDEX_INVALID;
					}
				}

				if (BRX_MOTION_UINT32_INDEX_INVALID != skeleton_a_joint_index)
				{
					DirectX::XMVECTOR unused_determinant;
					DirectX::XMFLOAT4X4 a_to_b_transform_model_space;
					DirectX::XMStoreFloat4x4(&a_to_b_transform_model_space, DirectX::XMMatrixMultiply(bind_pose_b_model_space[skeleton_b_joint_index], DirectX::XMMatrixInverse(&unused_determinant, bind_pose_a_model_space[skeleton_a_joint_index])));

					// Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
					a_to_b_transform_model_space.m[3][3] = 1.0f;

					direct_mappings.emplace_back(skeleton_a_joint_index, skeleton_b_joint_index, a_to_b_transform_model_space);

					// Mark elements mapped
					assert(!mapped_b[skeleton_b_joint_index]);
					mapped_b[skeleton_b_joint_index] = true;
				}
			}
		}

		brx_animation_skeleton_mapper_create_ragdoll_chain_mapping(
			skeleton_b.get_joint_count(),
			skeleton_b.get_joint_parent_indices(),
			this->m_ragdoll_mapping_data.get_direct_mappings(),
			this->m_ragdoll_mapping_data.access_chain_mappings());

		brx_animation_skeleton_mapper_create_ragdoll_unmapped(
			skeleton_b.get_joint_count(),
			skeleton_b.get_joint_parent_indices(),
			this->m_ragdoll_mapping_data.get_direct_mappings(),
			this->m_ragdoll_mapping_data.get_chain_mappings(),
			this->m_ragdoll_mapping_data.access_unmapped());

		if (inLockTranslations)
		{
			brx_animation_skeleton_mapper_create_ragdoll_locked_translation(
				skeleton_b.get_joint_count(),
				skeleton_b.get_joint_parent_indices(),
				this->m_ragdoll_mapping_data.get_direct_mappings(),
				this->m_ragdoll_mapping_data.access_locked_translations());
		}
	}

	JPH_ASSERT(mMappings.empty() && mChains.empty()); // Should not be initialized yet

	// Count joints
	int skeleton2_joint_count = inSkeleton2->GetJointCount();

	// Keep track of mapped joints (initialize to false)
	Array<bool> mapped2(skeleton2_joint_count, false);

	// Find joints that can be mapped directly
	for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
	{
		int skeleton1_joint_index = inMapJoint2To1(inSkeleton1, inSkeleton2, skeleton2_joint_index);
		if (-1 != skeleton1_joint_index)
		{
			// Calculate the transform that takes this joint from skeleton 1 to 2
			Mat44 joint_1_to_2 = inNeutralPose1[skeleton1_joint_index].Inversed() * inNeutralPose2[skeleton2_joint_index];

			// Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
			joint_1_to_2(3, 3) = 1.0f;

			mMappings.emplace_back(skeleton1_joint_index, skeleton2_joint_index, joint_1_to_2);
			JPH_ASSERT(!mapped2[skeleton2_joint_index]);
			mapped2[skeleton2_joint_index] = true;
		}
	}

	// Find joint chains
	{
		Array<Array<int>> skeleton2_children_joint_indices(skeleton2_joint_count);
		Array<UnorderedSet<int>> skeleton2_children_joint_index_set(skeleton2_joint_count);
		Array<int> skeleton2_depth_first_search_stack;
		Array<std::pair<int, int>> skeleton2_depth_first_search_ancestor_map_reverse_skeleton1;
		int skeleton2_depth_first_search_chain_start_ancestor_map_index = -1;
		for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
		{
			int skeleton2_parent_joint_index = inSkeleton2->GetJoints()[skeleton2_joint_index].mParentJointIndex;
			if (-1 != skeleton2_parent_joint_index)
			{
				skeleton2_children_joint_indices[skeleton2_parent_joint_index].push_back(skeleton2_joint_index);
				JPH_ASSERT(skeleton2_children_joint_index_set[skeleton2_parent_joint_index].end() == skeleton2_children_joint_index_set[skeleton2_parent_joint_index].find(skeleton2_joint_index));
				skeleton2_children_joint_index_set[skeleton2_parent_joint_index].emplace_hint(skeleton2_children_joint_index_set[skeleton2_parent_joint_index].end(), skeleton2_joint_index);
			}
			else
			{
				skeleton2_depth_first_search_stack.push_back(skeleton2_joint_index);
			}
		}
		JPH_ASSERT(1U == skeleton2_depth_first_search_stack.size());

		Array<bool> skeleton2_joint_visited_flags(static_cast<size_t>(skeleton2_joint_count), false);
		Array<bool> skeleton2_joint_pushed_flags(static_cast<size_t>(skeleton2_joint_count), false);
		while (!skeleton2_depth_first_search_stack.empty())
		{
			int skeleton2_current_joint_index = skeleton2_depth_first_search_stack.back();
			skeleton2_depth_first_search_stack.pop_back();

			JPH_ASSERT(!skeleton2_joint_visited_flags[skeleton2_current_joint_index]);
			skeleton2_joint_visited_flags[skeleton2_current_joint_index] = true;

			// Back Tracking
			bool back_tracking = false;
			while ((!skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.empty()) && (skeleton2_children_joint_index_set[skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.back().first].end() == skeleton2_children_joint_index_set[skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.back().first].find(skeleton2_current_joint_index)))
			{
				skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.pop_back();
				back_tracking = true;
			}

			if (back_tracking)
			{
				skeleton2_depth_first_search_chain_start_ancestor_map_index = -1;
				for (int chain_ancestor_map_index = static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()); chain_ancestor_map_index > 0; --chain_ancestor_map_index)
				{
					if (-1 != skeleton2_depth_first_search_ancestor_map_reverse_skeleton1[chain_ancestor_map_index - 1].second)
					{
						skeleton2_depth_first_search_chain_start_ancestor_map_index = (chain_ancestor_map_index - 1);
						break;
					}
				}
			}

			JPH_ASSERT((-1 == skeleton2_depth_first_search_chain_start_ancestor_map_index) || skeleton2_depth_first_search_chain_start_ancestor_map_index < static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()));

			int skeleton1_current_joint_index = inMapJoint2To1(inSkeleton1, inSkeleton2, skeleton2_current_joint_index);
			skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.push_back({skeleton2_current_joint_index, skeleton1_current_joint_index});

			if (-1 != skeleton1_current_joint_index)
			{
				if ((-1 != skeleton2_depth_first_search_chain_start_ancestor_map_index) && (static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()) > (skeleton2_depth_first_search_chain_start_ancestor_map_index + 2)))
				{
					// It should have joints between the mapped joints
					Array<int> chain2;
					chain2.reserve(static_cast<size_t>(static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()) - skeleton2_depth_first_search_chain_start_ancestor_map_index));
					for (int chain_ancestor_map_index = skeleton2_depth_first_search_chain_start_ancestor_map_index; chain_ancestor_map_index < int(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()); ++chain_ancestor_map_index)
					{
						chain2.push_back(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1[chain_ancestor_map_index].first);

						// Mark elements mapped
						JPH_ASSERT((skeleton2_depth_first_search_chain_start_ancestor_map_index == chain_ancestor_map_index) || ((static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()) - 1) == chain_ancestor_map_index) || (!mapped2[skeleton2_depth_first_search_ancestor_map_reverse_skeleton1[chain_ancestor_map_index].first]));
						mapped2[skeleton2_depth_first_search_ancestor_map_reverse_skeleton1[chain_ancestor_map_index].first] = true;
					}

#ifdef JPH_ENABLE_ASSERTS
					// Whether the chain exists in 1 too
					{
						int const chain1_start_joint_index = skeleton2_depth_first_search_ancestor_map_reverse_skeleton1[skeleton2_depth_first_search_chain_start_ancestor_map_index].second;
						int const chain1_end_joint_index = skeleton1_current_joint_index;
						JPH_ASSERT(-1 != chain1_start_joint_index);
						JPH_ASSERT(-1 != chain1_end_joint_index);
						int chain1_current_joint_index = chain1_end_joint_index;
						while ((-1 != chain1_current_joint_index) && (chain1_start_joint_index != chain1_current_joint_index))
						{
							chain1_current_joint_index = inSkeleton1->GetJoint(chain1_current_joint_index).mParentJointIndex;
						}
						JPH_ASSERT(chain1_start_joint_index == chain1_current_joint_index);
					}
#endif
					// Insert the chain
					mChains.emplace_back(std::move(chain2));
				}

				skeleton2_depth_first_search_chain_start_ancestor_map_index = static_cast<int>(skeleton2_depth_first_search_ancestor_map_reverse_skeleton1.size()) - 1;
			}

			for (int child_joint_index_index = static_cast<int>(skeleton2_children_joint_indices[skeleton2_current_joint_index].size()); child_joint_index_index > 0; --child_joint_index_index)
			{
				int const skeleton2_child_joint_index = skeleton2_children_joint_indices[skeleton2_current_joint_index][child_joint_index_index - 1];
				if ((!skeleton2_joint_visited_flags[skeleton2_child_joint_index]) && (!skeleton2_joint_pushed_flags[skeleton2_child_joint_index]))
				{
					skeleton2_joint_pushed_flags[skeleton2_child_joint_index] = true;
					skeleton2_depth_first_search_stack.push_back(skeleton2_child_joint_index);
				}
				else
				{
					JPH_ASSERT(false);
				}
			}
		}
	}

	// Collect unmapped joints from 2
	for (int skeleton2_joint_index = 0; skeleton2_joint_index < skeleton2_joint_count; ++skeleton2_joint_index)
	{
		if (!mapped2[skeleton2_joint_index])
		{
			mUnmapped.emplace_back(skeleton2_joint_index);
		}
	}

	if (inLockTranslations)
	{
		LockAllTranslations(inSkeleton2, inNeutralPose2);
	}
}

void SkeletonMapper::LockTranslations(const Skeleton *inSkeleton2, const bool *inLockedTranslations, const Mat44 *inNeutralPose2)
{
	JPH_ASSERT(inSkeleton2->AreJointsCorrectlyOrdered());

	int n = inSkeleton2->GetJointCount();

	// Copy locked joints to array but don't actually include the first joint (this is physics driven)
	for (int i = 0; i < n; ++i)
	{
		if (inLockedTranslations[i])
		{
			Locked l;
			l.mJointIdx = i;
			mLockedTranslations.push_back(l);
		}
	}
}

void SkeletonMapper::LockAllTranslations(const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2)
{
	JPH_ASSERT(!mMappings.empty(), "Call Initialize first!");
	JPH_ASSERT(inSkeleton2->AreJointsCorrectlyOrdered());

	// The first mapping is the top most one (remember that joints should be ordered so that parents go before children).
	// Because we created the mappings from the lowest joint first, this should contain the first mappable joint.
	int root_idx = mMappings[0].mJointIdx2;

	// Create temp array to hold locked joints
	int n = inSkeleton2->GetJointCount();
	bool *locked_translations = (bool *)JPH_STACK_ALLOC(n * sizeof(bool));
	memset(locked_translations, 0, n * sizeof(bool));

	// Mark root as locked
	locked_translations[root_idx] = true;

	// Loop over all joints and propagate the locked flag to all children
	for (int i = root_idx + 1; i < n; ++i)
	{
		int parent_idx = inSkeleton2->GetJoint(i).mParentJointIndex;
		if (parent_idx >= 0)
			locked_translations[i] = locked_translations[parent_idx];
	}

	// Unmark root because we don't actually want to include this (this determines the position of the entire ragdoll)
	locked_translations[root_idx] = false;

	// Call the generic function
	LockTranslations(inSkeleton2, locked_translations, inNeutralPose2);
}

void SkeletonMapper::MapModelSpace(const Mat44 *inPose1ModelSpace, const Skeleton *inSkeleton2, const Mat44 *inPose2LocalSpace, Mat44 *real_outPose2ModelSpace) const
{
	{
		mcrt_vector<uint32_t> skeleton_b_joint_parent_indices(static_cast<size_t>(inSkeleton2->GetJointCount()));
		for (int skeleton2_joint_index = 0; skeleton2_joint_index < inSkeleton2->GetJointCount(); ++skeleton2_joint_index)
		{
			skeleton_b_joint_parent_indices[skeleton2_joint_index] = inSkeleton2->GetJoint(skeleton2_joint_index).mParentJointIndex;
		}

		brx_animation_skeleton_mapper_map_pose_model_space(this->m_ragdoll_mapping_data, reinterpret_cast<DirectX::XMFLOAT4X4 const *>(inPose1ModelSpace), inSkeleton2->GetJointCount(), &skeleton_b_joint_parent_indices[0], reinterpret_cast<DirectX::XMFLOAT4X4 const *>(inPose2LocalSpace), reinterpret_cast<DirectX::XMFLOAT4X4 *>(real_outPose2ModelSpace));
	}

#ifdef JPH_ENABLE_ASSERTS
	bool *mapped2 = (bool *)JPH_STACK_ALLOC(sizeof(bool) * inSkeleton2->GetJointCount());
	memset(mapped2, 0, sizeof(bool) * inSkeleton2->GetJointCount());
#endif

	Mat44 *outPose2ModelSpace = (Mat44 *)JPH_STACK_ALLOC(sizeof(Mat44) * inSkeleton2->GetJointCount());

	// Apply direct mappings
	for (const Mapping &m : mMappings)
	{
#ifdef JPH_ENABLE_ASSERTS
		JPH_ASSERT(!mapped2[m.mJointIdx2]);
		mapped2[m.mJointIdx2] = true;
#endif
		outPose2ModelSpace[m.mJointIdx2] = inPose1ModelSpace[m.mJointIdx1] * m.mJoint1To2;
	}

	// Apply chain mappings
	for (const Chain &c : mChains)
	{
		// "mJoint1To2" missed in the original code
		// outPose2ModelSpace = inPose1ModelSpace * mJoint1To2
		JPH_ASSERT(mapped2[c.mJointIndices2.back()]);
		Vec3 in_target_position_model_space = outPose2ModelSpace[c.mJointIndices2.back()].GetTranslation();
		Mat44 in_end_effector_transform_local_space = inPose2LocalSpace[c.mJointIndices2.back()];

		DirectX::XMFLOAT4X4 *inout_chain_local_space = nullptr;
		DirectX::XMFLOAT4X4 *inout_chain_model_space = nullptr;
		{
			{
				size_t const alignment = alignof(DirectX::XMFLOAT4X4);
				size_t const size = sizeof(DirectX::XMFLOAT4X4) * (c.mJointIndices2.size() - 1);

				size_t space_local_space = size + (alignment - 1U);
				void *ptr_local_space = JPH_STACK_ALLOC(space_local_space);
				inout_chain_local_space = static_cast<DirectX::XMFLOAT4X4 *>(std::align(alignment, size, ptr_local_space, space_local_space));

				size_t space_model_space = size + (alignment - 1U);
				void *ptr_model_space = JPH_STACK_ALLOC(space_model_space);
				inout_chain_model_space = static_cast<DirectX::XMFLOAT4X4 *>(std::align(alignment, size, ptr_model_space, space_model_space));
			}

			inout_chain_local_space[0] = *reinterpret_cast<DirectX::XMFLOAT4X4 const *>(&inPose2LocalSpace[c.mJointIndices2.front()]);

			JPH_ASSERT(mapped2[c.mJointIndices2.front()]);
			Mat44 chain_end = outPose2ModelSpace[c.mJointIndices2.front()];
			DirectX::XMStoreFloat4x4(&inout_chain_model_space[0], DirectX::XMLoadFloat4x4(reinterpret_cast<DirectX::XMFLOAT4X4 const *>(&chain_end)));

			// Calculate end of chain given local space transforms of the joints of the chain
			for (int j = 1; j < (int)c.mJointIndices2.size() - 1; ++j)
			{
				Mat44 chain_end_local_space = inPose2LocalSpace[c.mJointIndices2[j]];
				inout_chain_local_space[j] = *reinterpret_cast<DirectX::XMFLOAT4X4 const *>(&chain_end_local_space);

				chain_end = chain_end * chain_end_local_space;
				DirectX::XMStoreFloat4x4(&inout_chain_model_space[j], DirectX::XMLoadFloat4x4(reinterpret_cast<DirectX::XMFLOAT4X4 const *>(&chain_end)));
			}
		}

		ik_reaching_solve(*reinterpret_cast<DirectX::XMFLOAT3 const *>(&in_target_position_model_space), *reinterpret_cast<DirectX::XMFLOAT4X4 const *>(&in_end_effector_transform_local_space), (uint32_t)c.mJointIndices2.size() - 1, &inout_chain_local_space[0], &inout_chain_model_space[0]);

		// Update all joints but the first and the last joint
		for (int j = 0; j < (int)c.mJointIndices2.size() - 1; ++j)
		{
#ifdef JPH_ENABLE_ASSERTS
			JPH_ASSERT((0 == j) || (!mapped2[c.mJointIndices2[j]]));
			mapped2[c.mJointIndices2[j]] = true;
#endif
			outPose2ModelSpace[c.mJointIndices2[j]] = (*reinterpret_cast<Mat44 *>(&inout_chain_model_space[j]));
		}
	}

	// All unmapped joints take the local pose and convert it to model space
	for (const Unmapped &u : mUnmapped)
	{
		const int JointIdx = u.mJointIdx;
		const int ParentJointIdx = inSkeleton2->GetJoint(JointIdx).mParentJointIndex;
		if (ParentJointIdx >= 0)
		{
#ifdef JPH_ENABLE_ASSERTS
			JPH_ASSERT(mapped2[ParentJointIdx]);
			JPH_ASSERT(!mapped2[JointIdx]);
			mapped2[JointIdx] = true;
#endif
			outPose2ModelSpace[JointIdx] = outPose2ModelSpace[ParentJointIdx] * inPose2LocalSpace[JointIdx];
		}
		else
		{
#ifdef JPH_ENABLE_ASSERTS
			JPH_ASSERT(!mapped2[JointIdx]);
			mapped2[JointIdx] = true;
#endif
			outPose2ModelSpace[JointIdx] = inPose2LocalSpace[JointIdx];
		}
	}

#ifdef JPH_ENABLE_ASSERTS
	for (int JointIdx = 0; JointIdx < inSkeleton2->GetJointCount(); ++JointIdx)
	{
		JPH_ASSERT(mapped2[JointIdx]);
	}
#endif

	// Update all locked joint translations
	for (const Locked &l : mLockedTranslations)
	{
		const int JointIdx = l.mJointIdx;
		const int ParentJointIdx = inSkeleton2->GetJoint(JointIdx).mParentJointIndex;

		JPH_ASSERT(ParentJointIdx < JointIdx);

		// use the current animation pose instead of the bind pose as the source
		outPose2ModelSpace[JointIdx].SetTranslation(outPose2ModelSpace[ParentJointIdx] * inPose2LocalSpace[JointIdx].GetTranslation());
	}

	for (int JointIdx = 0; JointIdx < inSkeleton2->GetJointCount(); ++JointIdx)
	{
		outPose2ModelSpace[JointIdx].IsClose(real_outPose2ModelSpace[JointIdx]);
	}
}

JPH_NAMESPACE_END
