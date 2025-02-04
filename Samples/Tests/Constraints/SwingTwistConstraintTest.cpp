// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/Constraints/SwingTwistConstraintTest.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Application/DebugUI.h>
#include <Layers.h>

JPH_IMPLEMENT_RTTI_VIRTUAL(SwingTwistConstraintTest)
{
	JPH_ADD_BASE_CLASS(SwingTwistConstraintTest, Test)
}

void SwingTwistConstraintTest::Initialize()
{
	JPH::Body *fixed_body;
	{
		JPH::Vec3 const half_extents(1.0F, 0.25F, 0.5F);

		JPH::Quat const rotation(0.0F, 0.0F, 0.0F, 1.0F);

		JPH::Vec3 const position(-2.0F, 0.0F, 0.0F);

		JPH::Shape *shape = new JPH::BoxShape(half_extents);

		JPH::BodyCreationSettings body_settings(shape, position, rotation, JPH::EMotionType::Static, Layers::NON_MOVING);

		fixed_body = mBodyInterface->CreateBody(body_settings);
	}

	JPH::Body *moveable_body;
	{
		JPH::Vec3 const half_extents(1.0F, 0.25F, 0.5F);

		JPH::Quat const rotation(0.0F, 0.0F, 0.0F, 1.0F);

		JPH::Vec3 const position(2.0F, 0.0F, 0.0F);

		JPH::Shape *shape = new JPH::BoxShape(half_extents);

		JPH::BodyCreationSettings body_settings(shape, position, rotation, JPH::EMotionType::Dynamic, Layers::MOVING);
		body_settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		body_settings.mMassPropertiesOverride.mMass = 10.0F;

		moveable_body = mBodyInterface->CreateBody(body_settings);
	}

	JPH::SwingTwistConstraint *constraint;
	{
		JPH::Vec3 const pivot(0.0f, 0.0f, 0.0f);
		JPH::Vec3 const twistAxis(1.0f, 0.0f, 0.0f);
		JPH::Vec3 const planeAxis(0.0f, 1.0f, 0.0f);

		float const twistMin = JPH::JPH_PI * -0.1F;
		float const twistMax = JPH::JPH_PI * 0.4F;
		float const planeMin = JPH::JPH_PI * -0.2F;
		float const planeMax = JPH::JPH_PI * 0.1F;
		float const cone = JPH::JPH_PI * 0.3F;

		sTwistMinAngle = twistMin;
		sTwistMaxAngle = twistMax;
		sPlaneHalfConeAngle = (planeMax - planeMin) * 0.5F;
		sNormalHalfConeAngle = cone;

		JPH::SwingTwistConstraintSettings constraint_settings;
		constraint_settings.SetEmbedded();

		constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
		constraint_settings.mPosition1 = pivot;
		constraint_settings.mPosition2 = pivot;
		constraint_settings.mTwistAxis1 = twistAxis;
		constraint_settings.mTwistAxis2 = twistAxis;
		constraint_settings.mPlaneAxis1 = planeAxis;
		constraint_settings.mPlaneAxis2 = planeAxis;

		constraint_settings.mTwistMinAngle = sTwistMinAngle;
		constraint_settings.mTwistMaxAngle = sTwistMaxAngle;
		constraint_settings.mPlaneHalfConeAngle = sPlaneHalfConeAngle;
		constraint_settings.mNormalHalfConeAngle = sNormalHalfConeAngle;

		constraint = static_cast<JPH::SwingTwistConstraint *>(constraint_settings.Create(*fixed_body, *moveable_body));
	}

	moveable_body->SetAllowSleeping(false);

	mBodyInterface->AddBody(fixed_body->GetID(), EActivation::Activate);
	mBodyInterface->AddBody(moveable_body->GetID(), EActivation::Activate);
	mPhysicsSystem->AddConstraint(constraint);

	mConstraint = constraint;
}

void SwingTwistConstraintTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	if (sTwistMinAngle <= sTwistMaxAngle)
	{
		mConstraint->SetTwistMinAngle(sTwistMinAngle);
		mConstraint->SetTwistMaxAngle(sTwistMaxAngle);
	}
	mConstraint->SetPlaneHalfConeAngle(sPlaneHalfConeAngle);
	mConstraint->SetNormalHalfConeAngle(sNormalHalfConeAngle);
}

void SwingTwistConstraintTest::CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu)
{
	inUI->CreateSlider(inSubMenu, "Min Twist Angle (deg)", RadiansToDegrees(sTwistMinAngle), -180.0f, 180.0f, 1.0f, [=](float inValue)
					   { sTwistMinAngle = DegreesToRadians(inValue); });
	inUI->CreateSlider(inSubMenu, "Max Twist Angle (deg)", RadiansToDegrees(sTwistMaxAngle), -180.0f, 180.0f, 1.0f, [=](float inValue)
					   { sTwistMaxAngle = DegreesToRadians(inValue); });
	inUI->CreateSlider(inSubMenu, "Normal Half Cone Angle (deg)", RadiansToDegrees(sNormalHalfConeAngle), 0.0f, 180.0f, 1.0f, [=](float inValue)
					   { sNormalHalfConeAngle = DegreesToRadians(inValue); });
	inUI->CreateSlider(inSubMenu, "Plane Half Cone Angle (deg)", RadiansToDegrees(sPlaneHalfConeAngle), 0.0f, 180.0f, 1.0f, [=](float inValue)
					   { sPlaneHalfConeAngle = DegreesToRadians(inValue); });

	inUI->ShowMenu(inSubMenu);
}
