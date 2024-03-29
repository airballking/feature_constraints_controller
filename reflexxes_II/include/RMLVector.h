//  ---------------------- Doxygen info ----------------------
//! \file RMLVector.h
//!
//! \brief
//! Header file for the dynamic vector class used for the Reflexxes Motion Libraries
//!
//! \details
//! This file implements a minimalist version of a dynamic vector class
//! to be used for the interface of the Reflexxes On-Line Trajectory
//! Generation algorithm. Furthermore, three type-specific versions 
//! of this class are defined:
//! 
//!  - RMLDoubleVector for \c double values,
//! 
//!	 - RMLIntVector for \c integer values, and
//! 
//!  - RMLBoolVector for \c bool values.
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
//!	\author Torsten Kroeger, <info@reflexxes.com>
//!	
//!
//! \note Copyright (C) 2012 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __RMLVector__
#define __RMLVector__


#include <string.h>

//  ---------------------- Doxygen info ----------------------
//! \class RMLVector
//!
//! \brief
//! This is a minimalistic dynamic vector class implementation
//! used for the Reflexxes Motion Libraries
//! 
//! \note
//! For all copy and comparison methods of this class, no check, 
//! whether two objects are of the same size, is implemented.
//! In order to safe computation time, it is assumed that the used
//! RMLVector objects are of the same size. Ensure that only
//! RMLVector objects of equal size are used.
//! 
//! \sa RMLDoubleVector
//! \sa RMLIntVector
//! \sa RMLBoolVector
//  ----------------------------------------------------------
template <class T = double>
class RMLVector
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const RMLVector<T>& Vector)
//! 
//! \brief
//! Copy constructor of class RMLVector
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated. 
//! 
//! \param Vector
//! Original object reference
//  ----------------------------------------------------------
    RMLVector(const RMLVector<T> &Vector)
    {
        this->VectorDimension		=	Vector.GetVecDim()				;
        this->VecData				=	new T[this->VectorDimension]	;
        *this						=	Vector							;		
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const unsigned int Size)
//! 
//! \brief
//! Constructor of class RMLVector, allocates memory for a given
//! number of double elements
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Size
//! Determines the number of vector elements
//  ----------------------------------------------------------
    RMLVector(const unsigned int Size)
    {

        this->VectorDimension		=	Size							;
        this->VecData				=	new T[this->VectorDimension]	;

        memset(		this->VecData
                ,	0x0
                ,	(this->VectorDimension * sizeof(T))	);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1)
//! 
//! \brief
//! Special 2D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1	)
    {
        this->VectorDimension		=	2									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1, const T &Component2)
//! 
//! \brief
//! Special 3D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1
                ,	const T &Component2	)
    {
        this->VectorDimension		=	3									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
        this->VecData[2]			=	Component2							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3)
//! 
//! \brief
//! Special 4D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1
                ,	const T &Component2
                ,	const T &Component3	)
    {
        this->VectorDimension		=	4									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
        this->VecData[2]			=	Component2							;
        this->VecData[3]			=	Component3							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4)
//! 
//! \brief Special 5D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1
                ,	const T &Component2
                ,	const T &Component3
                ,	const T &Component4	)
    {
        this->VectorDimension		=	5									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
        this->VecData[2]			=	Component2							;
        this->VecData[3]			=	Component3							;
        this->VecData[4]			=	Component4							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4, const T &Component5)
//! 
//! \brief
//! Special 6D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//! 
//! \param Component5
//! Value of the sixth vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1
                ,	const T &Component2
                ,	const T &Component3
                ,	const T &Component4
                ,	const T &Component5	)
    {
        this->VectorDimension		=	6									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
        this->VecData[2]			=	Component2							;
        this->VecData[3]			=	Component3							;
        this->VecData[4]			=	Component4							;
        this->VecData[5]			=	Component5							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn RMLVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4, const T &Component5, const T &Component6)
//! 
//! \brief
//! Special 7D constructor
//! 
//! \warning
//! This method is \b not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//! 
//! \param Component5
//! Value of the sixth vector component
//! 
//! \param Component6
//! Value of the seventh vector component
//  ----------------------------------------------------------
    RMLVector(		const T &Component0
                ,	const T &Component1
                ,	const T &Component2
                ,	const T &Component3
                ,	const T &Component4
                ,	const T &Component5
                ,	const T &Component6	)
    {
        this->VectorDimension		=	7									;
        this->VecData				=	(T*) new T[this->VectorDimension]	;

        this->VecData[0]			=	Component0							;
        this->VecData[1]			=	Component1							;
        this->VecData[2]			=	Component2							;
        this->VecData[3]			=	Component3							;
        this->VecData[4]			=	Component4							;
        this->VecData[5]			=	Component5							;
        this->VecData[6]			=	Component6							;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn ~RMLVector(void)
//! 
//! \brief
//! Destructor of class RMLVector
//  ----------------------------------------------------------
    ~RMLVector(void)
    {
        delete[] this->VecData;
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline void Set(const T Value)
//! 
//! \brief Sets all elements of a vector of double elements to one specific value
//! 
//! \param Value
//! Value for all elements of the vector
//  ----------------------------------------------------------
    inline void Set(const T Value)
    {
        unsigned int i;

        for( i = 0; i < this->VectorDimension; i++)
        {
            this->VecData[i] = Value;
        }
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline RMLVector &operator = (const RMLVector<T>& Vector)
//! 
//! \brief
//! Copy operator
//! 
//! \param Vector
//! Vector object to be copied
//  ----------------------------------------------------------
    inline RMLVector &operator = (const RMLVector<T>& Vector)
    {
        memcpy(		(void*)(this->VecData)
                ,	(void*)(Vector.VecData)
                ,	(this->VectorDimension * sizeof(T))	);

        return(*this);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline T& operator [] (const int Index)
//! 
//! \brief
//! Bracket operator, gives access to a single vector element
//! 
//! \param Index
//! Determines the desired vector element
//! 
//! \return
//! Reference to one single vector element
//  ----------------------------------------------------------
    inline T& operator [] (const int Index)
    {
        return(this->VecData[Index]);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline const T& operator [] (const int Index) const
//! 
//! \brief
//! Bracket operator, gives access to a single vector element
//! 
//! \param Index
//! Determines the desired vector element
//! 
//! \return
//! Constant pointer to one single vector element
//  ----------------------------------------------------------
    inline const T& operator [] (const int Index) const
    {
        return(this->VecData[Index]);
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator == (const RMLVector<T> &Vector) const
//! 
//! \brief
//! Equal operator
//! 
//! \return
//! \c true if all vector elements are equal or \c false in all other cases
//  ----------------------------------------------------------
    inline bool operator == (const RMLVector<T> &Vector) const
    {
        unsigned int i;

        for( i = 0; i < this->VectorDimension; i++)
        {
            if( (*this)[i] != Vector[i] )
            {
                return(false); // vector components !=
            }
        }
        return(true); // all vector components ==
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline bool operator != (const RMLVector<T> &Vector) const
//! 
//! \brief
//! Unequal operator
//! \return
//! \c true if all vector elements are equal and \c false in all other cases
//  ----------------------------------------------------------
    inline bool operator != (const RMLVector<T> &Vector) const
    {
        return(!(*this == Vector));
    }


//  ---------------------- Doxygen info ----------------------
//! \fn inline unsigned int GetVecDim(void) const
//! 
//! \brief
//! Returns the dimension of the vector
//! 
//! \return
//! Number of vector elements
//  ----------------------------------------------------------
    inline unsigned int GetVecDim(void) const
    {
        return(this->VectorDimension);
    }

//  ---------------------- Doxygen info ----------------------
//! \fn inline T* GetReference(void) const
//! 
//! \brief
//! Returns the \em data pointer of the vector object (not the pointer to the \em object)
//! 
//! \return
//! Data pointer
//  ----------------------------------------------------------
    inline T* GetReference(void) const
    {
        return((T*)(&(this->VecData[0])));
    }


//  ---------------------- Doxygen info ----------------------
//! \var T *VecData
//! 
//! \brief
//! Pointer to the actual vector data, that is, an array of type \c T
//! objects
//  ----------------------------------------------------------
    T				*VecData;	

    
//  ---------------------- Doxygen info ----------------------
//! \var unsigned int VectorDimension
//! 
//! \brief
//! Contains the number of vector elements
//  ----------------------------------------------------------
    unsigned int	VectorDimension;



};	// class RMLVector


//  ---------------------- Doxygen info ----------------------
//! \typedef RMLDoubleVector
//!
//! \brief
//! Type definition for vectors of \c double elements
//!
//! \sa RMLVector
//  ----------------------------------------------------------
typedef	RMLVector<double>	RMLDoubleVector;


//  ---------------------- Doxygen info ----------------------
//! \typedef RMLIntVector
//!
//! \brief
//! Type definition for vectors of \c int elements
//!
//! \sa RMLVector
//  ----------------------------------------------------------
typedef	RMLVector<int>		RMLIntVector;


//  ---------------------- Doxygen info ----------------------
//! \typedef RMLBoolVector
//!
//! \brief
//! Type definition for vectors of \c bool elements
//!
//! \sa RMLVector
//  ----------------------------------------------------------
typedef	RMLVector<bool>		RMLBoolVector;



#endif
