
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_rgbd_image_message_t.idl using "rtiddsgen".
  The rtiddsgen tool is part of the RTI Data Distribution Service distribution.
  For more information, type 'rtiddsgen -help' at a command shell
  or consult the RTI Data Distribution Service manual.
*/

#ifndef dds_rgbd_image_message_t_637283907_h
#define dds_rgbd_image_message_t_637283907_h

#ifndef NDDS_STANDALONE_TYPE
    #ifdef __cplusplus
        #ifndef ndds_cpp_h
            #include "ndds/ndds_cpp.h"
        #endif
    #else
        #ifndef ndds_c_h
            #include "ndds/ndds_c.h"
        #endif
    #endif
#else
    #include "ndds_standalone_type.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

        
extern const char *dds_rgbd_image_message_tTYPENAME;
        

#ifdef __cplusplus
}
#endif

typedef struct dds_rgbd_image_message_t
{
    DDS_Long  camera_config;
    DDS_Long  camera_type;
    DDS_UnsignedLongLong  timestamp;
    DDS_Float  roll;
    DDS_Float  pitch;
    DDS_Float  yaw;
    DDS_Float  ground_x;
    DDS_Float  ground_y;
    DDS_Float  ground_z;
    DDS_Float  camera_matrix[9];
    DDS_Long  cols;
    DDS_Long  rows;
    DDS_Long  step1;
    DDS_Long  type1;
     DDS_CharSeq  imageData1;
    DDS_Long  step2;
    DDS_Long  type2;
     DDS_CharSeq  imageData2;

} dds_rgbd_image_message_t;
    
                            
#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
  /* If the code is building on Windows, start exporting symbols.
   */
  #undef NDDSUSERDllExport
  #define NDDSUSERDllExport __declspec(dllexport)
#endif

    
NDDSUSERDllExport DDS_TypeCode* dds_rgbd_image_message_t_get_typecode(void); /* Type code */
    

DDS_SEQUENCE(dds_rgbd_image_message_tSeq, dds_rgbd_image_message_t);
        
NDDSUSERDllExport
RTIBool dds_rgbd_image_message_t_initialize(
        dds_rgbd_image_message_t* self);
        
NDDSUSERDllExport
RTIBool dds_rgbd_image_message_t_initialize_ex(
        dds_rgbd_image_message_t* self,RTIBool allocatePointers);

NDDSUSERDllExport
void dds_rgbd_image_message_t_finalize(
        dds_rgbd_image_message_t* self);
                        
NDDSUSERDllExport
void dds_rgbd_image_message_t_finalize_ex(
        dds_rgbd_image_message_t* self,RTIBool deletePointers);
        
NDDSUSERDllExport
RTIBool dds_rgbd_image_message_t_copy(
        dds_rgbd_image_message_t* dst,
        const dds_rgbd_image_message_t* src);

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
  /* If the code is building on Windows, stop exporting symbols.
   */
  #undef NDDSUSERDllExport
  #define NDDSUSERDllExport
#endif



#endif /* dds_rgbd_image_message_t_637283907_h */
