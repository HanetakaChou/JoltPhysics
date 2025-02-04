// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Tests/Constraints/SixDOFConstraintTest.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Application/DebugUI.h>
#include <Layers.h>

JPH_IMPLEMENT_RTTI_VIRTUAL(SixDOFConstraintTest)
{
	JPH_ADD_BASE_CLASS(SixDOFConstraintTest, Test)
}

float SixDOFConstraintTest::sLimitMin[EAxis::Num] = {0, 0, 0, 0, 0, 0};
float SixDOFConstraintTest::sLimitMax[EAxis::Num] = {0, 0, 0, 0, 0, 0};

SixDOFConstraintTest::SettingsRef SixDOFConstraintTest::sSettings = []()
{
	static SixDOFConstraintSettings settings;
	settings.SetEmbedded();
	settings.mAxisX1 = settings.mAxisX2 = -Vec3::sAxisY();
	settings.mAxisY1 = settings.mAxisY2 = Vec3::sAxisZ();
	for (int i = 0; i < 6; ++i)
		settings.mMotorSettings[i] = MotorSettings(10.0f, 2.0f);
	return &settings;
}();

void SixDOFConstraintTest::Initialize()
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

	{
		JPH::Vec3 const pivot(0.0f, 0.0f, 0.0f);
		JPH::Vec3 const twistAxis(1.0f, 0.0f, 0.0f);
		JPH::Vec3 const planeAxis(0.0f, 1.0f, 0.0f);

		float const twistMin = JPH::JPH_PI * -0.1F;
		float const twistMax = JPH::JPH_PI * 0.4F;
		float const planeMin = JPH::JPH_PI * -0.2F;
		float const planeMax = JPH::JPH_PI * 0.1F;
		float const cone = JPH::JPH_PI * 0.3F;

		sSettings->mSpace = JPH::EConstraintSpace::WorldSpace;
		sSettings->mPosition1 = pivot;
		sSettings->mPosition2 = pivot;
		sSettings->mAxisX1 = twistAxis;
		sSettings->mAxisX2 = twistAxis;
		sSettings->mAxisY1 = planeAxis;
		sSettings->mAxisY2 = planeAxis;

		sLimitMin[JPH::SixDOFConstraintSettings::RotationX] = twistMin;
		sLimitMax[JPH::SixDOFConstraintSettings::RotationX] = twistMax;
		sLimitMin[JPH::SixDOFConstraintSettings::RotationY] = planeMin;
		sLimitMax[JPH::SixDOFConstraintSettings::RotationY] = planeMax;
		sLimitMin[JPH::SixDOFConstraintSettings::RotationZ] = -cone;
		sLimitMax[JPH::SixDOFConstraintSettings::RotationZ] = cone;

		for (int i = 0; i < JPH::SixDOFConstraintSettings::Num; ++i)
		{
			sSettings->SetLimitedAxis((JPH::SixDOFConstraintSettings::EAxis)i, sLimitMin[i], sLimitMax[i]);
		}

		mConstraint = static_cast<JPH::SixDOFConstraint *>(sSettings->Create(*fixed_body, *moveable_body));
	}

	moveable_body->SetAllowSleeping(false);

	mBodyInterface->AddBody(fixed_body->GetID(), EActivation::Activate);
	mBodyInterface->AddBody(moveable_body->GetID(), EActivation::Activate);
	mPhysicsSystem->AddConstraint(mConstraint);
}

void SixDOFConstraintTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	JPH::Vec3 translation_limit_min(sLimitMin[JPH::SixDOFConstraintSettings::TranslationX], sLimitMin[JPH::SixDOFConstraintSettings::TranslationY], sLimitMin[JPH::SixDOFConstraintSettings::TranslationZ]);
	JPH::Vec3 translation_limit_max(sLimitMax[JPH::SixDOFConstraintSettings::TranslationX], sLimitMax[JPH::SixDOFConstraintSettings::TranslationY], sLimitMax[JPH::SixDOFConstraintSettings::TranslationZ]);
	mConstraint->SetTranslationLimits(translation_limit_min, translation_limit_max);

	JPH::Vec3 rotation_limit_min(sLimitMin[JPH::SixDOFConstraintSettings::RotationX], sLimitMin[JPH::SixDOFConstraintSettings::RotationY], sLimitMin[JPH::SixDOFConstraintSettings::RotationZ]);
	JPH::Vec3 rotation_limit_max(sLimitMax[JPH::SixDOFConstraintSettings::RotationX], sLimitMax[JPH::SixDOFConstraintSettings::RotationY], sLimitMax[JPH::SixDOFConstraintSettings::RotationZ]);
	mConstraint->SetRotationLimits(rotation_limit_min, rotation_limit_max);
}

void SixDOFConstraintTest::CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu)
{
	Array<String> const labels = {"Translation X", "Translation Y", "Translation Z", "Rotation X", "Rotation Y", "Rotation Z"};

	for (int i = 0; i < 3; ++i)
	{
		inUI->CreateStaticText(inSubMenu, "Enable Limits " + labels[i]);
		inUI->CreateSlider(inSubMenu, "Limit Min", sLimitMin[i], -5.0f, 5.0f, 0.1f, [=](float inValue)
						   { sLimitMin[i] = inValue; });
		inUI->CreateSlider(inSubMenu, "Limit Max", sLimitMax[i], -5.0f, 5.0f, 0.1f, [=](float inValue)
						   { sLimitMax[i] = inValue; });
	}

	for (int i = 3; i < 6; ++i)
	{
		inUI->CreateStaticText(inSubMenu, "Enable Limits " + labels[i]);
		inUI->CreateSlider(inSubMenu, "Limit Min", RadiansToDegrees(sLimitMin[i]), -180.0f, 180.0f, 1.0f, [=](float inValue)
						   { sLimitMin[i] = DegreesToRadians(inValue); });
		inUI->CreateSlider(inSubMenu, "Limit Max", RadiansToDegrees(sLimitMax[i]), -180.0f, 180.0f, 1.0f, [=](float inValue)
						   { sLimitMax[i] = DegreesToRadians(inValue); });
	}

	inUI->ShowMenu(inSubMenu);
}
