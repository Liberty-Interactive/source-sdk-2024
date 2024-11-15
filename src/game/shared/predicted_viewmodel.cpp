//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
//=============================================================================//
#include "cbase.h"
#include "predicted_viewmodel.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

LINK_ENTITY_TO_CLASS( predicted_viewmodel, CPredictedViewModel );

IMPLEMENT_NETWORKCLASS_ALIASED( PredictedViewModel, DT_PredictedViewModel )

BEGIN_NETWORK_TABLE( CPredictedViewModel, DT_PredictedViewModel )
END_NETWORK_TABLE()

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
#ifdef CLIENT_DLL
CPredictedViewModel::CPredictedViewModel() : m_LagAnglesHistory("CPredictedViewModel::m_LagAnglesHistory")
{
	m_vLagAngles.Init();
	m_LagAnglesHistory.Setup( &m_vLagAngles, 0 );
}
#else
CPredictedViewModel::CPredictedViewModel()
{
}
#endif


//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
CPredictedViewModel::~CPredictedViewModel()
{
}

extern ConVar r_maxvmlag;

void CPredictedViewModel::CalcViewModelLag( Vector& origin, QAngle& angles, QAngle& original_angles )
{
#ifdef CLIENT_DLL
	Vector vOriginalOrigin = origin;
	QAngle vOriginalAngles = angles;

	// Calculate our drift
	Vector	forward;
	AngleVectors(angles, &forward, NULL, NULL);

	if (gpGlobals->frametime != 0.0f)
	{
		Vector vDifference;
		VectorSubtract(forward, m_vecLastFacing, vDifference);

		float flSpeed = 5.0f;

		// If we start to lag too far behind, we'll increase the "catch up" speed.  Solves the problem with fast cl_yawspeed, m_yaw or joysticks
		//  rotating quickly.  The old code would slam lastfacing with origin causing the viewmodel to pop to a new position
		float flDiff = vDifference.Length();
		if ((flDiff > r_maxvmlag.GetFloat()) && (r_maxvmlag.GetFloat() > 0.0f))
		{
			float flScale = flDiff / r_maxvmlag.GetFloat();
			flSpeed *= flScale;
		}

		// FIXME:  Needs to be predictable?
		VectorMA(m_vecLastFacing, flSpeed * gpGlobals->frametime, vDifference, m_vecLastFacing);
		// Make sure it doesn't grow out of control!!!
		VectorNormalize(m_vecLastFacing);
		VectorMA(origin, 5.0f, vDifference * -1.0f, origin);

		Assert(m_vecLastFacing.IsValid());
	}

	Vector right, up;
	AngleVectors(original_angles, &forward, &right, &up);

	float pitch = original_angles[PITCH];
	if (pitch > 180.0f)
		pitch -= 360.0f;
	else if (pitch < -180.0f)
		pitch += 360.0f;

	if (r_maxvmlag.GetFloat() == 0.0f)
	{
		origin = vOriginalOrigin;
		angles = vOriginalAngles;
	}

	//FIXME: These are the old settings that caused too many exposed polys on some models
	VectorMA(origin, -pitch * 0.035f, forward, origin);
	VectorMA(origin, -pitch * 0.03f, right, origin);
	VectorMA(origin, -pitch * 0.02f, up, origin);
#endif
}