//  ---------------------- Doxygen info ----------------------
//! \file TypeIIRMLStep2IntermediateProfiles.cpp
//!
//! \brief
//! Implementation file for the Step 2 intermediate velocity profiles of
//! the Type II On-Line Trajectory Generation algorithm
//!
//! \details
//! For further information, please refer to the file
//! TypeIIRMLStep2IntermediateProfiles.h.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see 
//! <http://www.gnu.org/licenses/>.
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date October 2012
//! 
//! \version 1.2.3
//!
//!	\author Torsten Kroeger, <info@reflexxes.com> \n
//!	
//!
//! \note Copyright (C) 2012 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------

#include <TypeIIRMLStep2IntermediateProfiles.h>
#include <TypeIIRMLPolynomial.h>
#include <TypeIIRMLMath.h>
#include <TypeIIRMLStep1IntermediateProfiles.h>

using namespace TypeIIRMLMath;


//****************************************************************************
// NegateStep2()

void TypeIIRMLMath::NegateStep2(	double				*ThisCurrentPosition
								,	double				*ThisCurrentVelocity
								,	double				*ThisTargetPosition
								,	double				*ThisTargetVelocity
								,	bool				*Inverted				)
{
	*ThisCurrentPosition	=	-(*ThisCurrentPosition	)	;
	*ThisCurrentVelocity	=	-(*ThisCurrentVelocity	)	;
	*ThisTargetPosition		=	-(*ThisTargetPosition	)	;
	*ThisTargetVelocity		=	-(*ThisTargetVelocity	)	;
	*Inverted				=	!(*Inverted)				;

    return;
}


//****************************************************************************
// VToVMaxStep2()

void TypeIIRMLMath::VToVMaxStep2(		double				*ThisCurrentTime
									,	double				*ThisCurrentPosition
									,	double				*ThisCurrentVelocity
									,	const double		&MaxVelocity
									,	const double		&MaxAcceleration
									,	MotionPolynomials	*PolynomialsLocal
									,	const bool			&Inverted				)
{
	double	TimeForCurrentStep	=	(*ThisCurrentVelocity - MaxVelocity)
									/ MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-(*ThisCurrentVelocity)), (-(*ThisCurrentPosition)), (*ThisCurrentTime)); 
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-(*ThisCurrentVelocity)), (*ThisCurrentTime));
		PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, (*ThisCurrentTime));
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (*ThisCurrentVelocity), (*ThisCurrentPosition), (*ThisCurrentTime)); 
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (*ThisCurrentVelocity), (*ThisCurrentTime));
		PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), (*ThisCurrentTime));
	}

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = (*ThisCurrentTime) + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++; 

    //calculate	values for the end of the current time interval	
    *ThisCurrentTime		+=	(TimeForCurrentStep);
	*ThisCurrentPosition	+=	0.5 * (*ThisCurrentVelocity + MaxVelocity)
								* TimeForCurrentStep;
	*ThisCurrentVelocity	=	MaxVelocity;

    return;
}


//****************************************************************************
// VToZeroStep2()

void TypeIIRMLMath::VToZeroStep2(		double				*ThisCurrentTime
									,	double				*ThisCurrentPosition
									,	double				*ThisCurrentVelocity
									,	const double		&MaxAcceleration
									,	MotionPolynomials	*PolynomialsLocal
									,	const bool			&Inverted				)
{
	double	TimeForCurrentStep	=	*ThisCurrentVelocity / MaxAcceleration;

    if(Inverted)
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * MaxAcceleration), (-(*ThisCurrentVelocity)), (-(*ThisCurrentPosition)), (*ThisCurrentTime)); 
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, MaxAcceleration , (-(*ThisCurrentVelocity)), (*ThisCurrentTime));
		PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , MaxAcceleration, (*ThisCurrentTime));
    }
    else
    {
        PolynomialsLocal->PositionPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients((0.5 * (-MaxAcceleration)), (*ThisCurrentVelocity), (*ThisCurrentPosition), (*ThisCurrentTime)); 
        PolynomialsLocal->VelocityPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, (-MaxAcceleration) , (*ThisCurrentVelocity), (*ThisCurrentTime));
		PolynomialsLocal->AccelerationPolynomial[PolynomialsLocal->ValidPolynomials].SetCoefficients(0.0, 0.0 , (-MaxAcceleration), (*ThisCurrentTime));
	}

    PolynomialsLocal->PolynomialTimes[PolynomialsLocal->ValidPolynomials] = (*ThisCurrentTime) + TimeForCurrentStep;
    PolynomialsLocal->ValidPolynomials++; 

    //calculate	values for the end of the current time interval	
    *ThisCurrentTime		+=	(TimeForCurrentStep);
	*ThisCurrentPosition	+=	0.5 * (*ThisCurrentVelocity) * TimeForCurrentStep;
	*ThisCurrentVelocity	=	0.0;

    return;
}