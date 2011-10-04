
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_obstacle_map_message_t.idl using "rtiddsgen".
  The rtiddsgen tool is part of the RTI Data Distribution Service distribution.
  For more information, type 'rtiddsgen -help' at a command shell
  or consult the RTI Data Distribution Service manual.
*/

#ifndef dds_obstacle_map_message_tSupport_1684969828_h
#define dds_obstacle_map_message_tSupport_1684969828_h

/* Uses */
#include "dds_obstacle_map_message_t.h"



#ifdef __cplusplus
#ifndef ndds_cpp_h
  #include "ndds/ndds_cpp.h"
#endif
#else
#ifndef ndds_c_h
  #include "ndds/ndds_c.h"
#endif
#endif

        

/* ========================================================================= */
/**
   Uses:     T

   Defines:  TTypeSupport, TDataWriter, TDataReader

   Organized using the well-documented "Generics Pattern" for
   implementing generics in C and C++.
*/

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
  /* If the code is building on Windows, start exporting symbols.
   */
  #undef NDDSUSERDllExport
  #define NDDSUSERDllExport __declspec(dllexport)

#ifdef __cplusplus
  /* If we're building on Windows, explicitly import the superclasses of
   * the types declared below.
   */        
  class __declspec(dllimport) DDSTypeSupport;
  class __declspec(dllimport) DDSDataWriter;
  class __declspec(dllimport) DDSDataReader;
#endif

#endif

#ifdef __cplusplus

DDS_TYPESUPPORT_CPP(dds_obstacle_map_message_tTypeSupport, dds_obstacle_map_message_t);

DDS_DATAWRITER_CPP(dds_obstacle_map_message_tDataWriter, dds_obstacle_map_message_t);
DDS_DATAREADER_CPP(dds_obstacle_map_message_tDataReader, dds_obstacle_map_message_tSeq, dds_obstacle_map_message_t);


#else

DDS_TYPESUPPORT_C(dds_obstacle_map_message_tTypeSupport, dds_obstacle_map_message_t);
DDS_DATAWRITER_C(dds_obstacle_map_message_tDataWriter, dds_obstacle_map_message_t);
DDS_DATAREADER_C(dds_obstacle_map_message_tDataReader, dds_obstacle_map_message_tSeq, dds_obstacle_map_message_t);

#endif

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
  /* If the code is building on Windows, stop exporting symbols.
   */
  #undef NDDSUSERDllExport
  #define NDDSUSERDllExport
#endif



#endif  /* dds_obstacle_map_message_tSupport_1684969828_h */
