/************************************************************************

*  Copyright (C) Xintu Photonics Co.,Ltd. 2012-2025. All rights reserved.

*  @file      MPTiff.h

*  @brief     MPTiff export functions header file	

*  @version	  1.0.0.1	    

*  @author    FDY

*  @date      2023-02-21 

************************************************************************/

#ifdef MPTIFF_LIB_EXPORTS
#define CMPTIFF_LIB_API extern "C" __declspec(dllexport)
#else
#define CMPTIFF_LIB_API extern "C" __declspec(dllimport)
#endif

CMPTIFF_LIB_API	bool CreatMPFile(char *pstrPath );      /* Creat IMG File */

CMPTIFF_LIB_API	bool CombMPFile(
		const unsigned char	*pBuffer,	    /*	[in] the IMG Buffer		*/
		int					width,			/*	[in] the IMG Width			*/
		int					height,			/*	[in] the IMG Height			*/
		int					channels,		/*	[in] the IMG Channel 1\3			*/
		int					depth			/*	[in] the IMG Depth Data 8\16	*/
		);

CMPTIFF_LIB_API	void CloseMPFile();                     /*	Close File	*/
