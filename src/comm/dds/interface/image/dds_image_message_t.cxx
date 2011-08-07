
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_image_message_t.idl using "rtiddsgen".
  The rtiddsgen tool is part of the RTI Data Distribution Service distribution.
  For more information, type 'rtiddsgen -help' at a command shell
  or consult the RTI Data Distribution Service manual.
*/


#ifndef NDDS_STANDALONE_TYPE
    #ifdef __cplusplus
        #ifndef ndds_cpp_h
            #include "ndds/ndds_cpp.h"
        #endif
        #ifndef dds_c_log_impl_h              
            #include "dds_c/dds_c_log_impl.h"                                
        #endif        
    #else
        #ifndef ndds_c_h
            #include "ndds/ndds_c.h"
        #endif
    #endif
    
    #ifndef cdr_type_h
        #include "cdr/cdr_type.h"
    #endif    

    #ifndef osapi_heap_h
        #include "osapi/osapi_heap.h" 
    #endif
#else
    #include "ndds_standalone_type.h"
#endif



#include "dds_image_message_t.h"

/* ========================================================================= */
const char *dds_image_message_tTYPENAME = "dds_image_message_t";

DDS_TypeCode* dds_image_message_t_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode dds_image_message_t_g_tc_imageData1_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(1228800,NULL);
    static DDS_TypeCode dds_image_message_t_g_tc_imageData2_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(1228800,NULL);

    static DDS_TypeCode_Member dds_image_message_t_g_tc_members[9]=
    {
        {
            (char *)"camera_type",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"cols",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"rows",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"step1",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"type1",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"imageData1",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"step2",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"type2",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        },
        {
            (char *)"imageData2",/* Member name */
            {
                0,/* Representation ID */
                DDS_BOOLEAN_FALSE,/* Is a pointer? */
                -1, /* Bitfield bits */
                NULL/* Member type code is assigned later */
            },
            0, /* Ignored */
            0, /* Ignored */
            0, /* Ignored */
            NULL, /* Ignored */
            DDS_BOOLEAN_FALSE, /* Is a key? */
            DDS_PRIVATE_MEMBER,/* Ignored */
            0,/* Ignored */
            NULL/* Ignored */
        }
    };

    static DDS_TypeCode dds_image_message_t_g_tc =
    {{
        DDS_TK_STRUCT,/* Kind */
        DDS_BOOLEAN_FALSE, /* Ignored */
        -1,/* Ignored */
        (char *)"dds_image_message_t", /* Name */
        NULL, /* Ignored */
        0, /* Ignored */
        0, /* Ignored */
        NULL, /* Ignored */
        9, /* Number of members */
        dds_image_message_t_g_tc_members, /* Members */
        DDS_VM_NONE /* Ignored */
    }}; /* Type code for dds_image_message_t*/

    if (is_initialized) {
        return &dds_image_message_t_g_tc;
    }

    dds_image_message_t_g_tc_imageData1_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_image_message_t_g_tc_imageData2_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;

    dds_image_message_t_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&dds_image_message_t_g_tc_imageData1_sequence;
    dds_image_message_t_g_tc_members[6]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[7]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_image_message_t_g_tc_members[8]._representation._typeCode = (RTICdrTypeCode *)&dds_image_message_t_g_tc_imageData2_sequence;

    is_initialized = RTI_TRUE;

    return &dds_image_message_t_g_tc;
}


RTIBool dds_image_message_t_initialize(
    dds_image_message_t* sample) {
  return dds_image_message_t_initialize_ex(sample,RTI_TRUE);
}
        
RTIBool dds_image_message_t_initialize_ex(
    dds_image_message_t* sample,RTIBool allocatePointers)
{

    void* buffer;                
    buffer = NULL;        

    if (!RTICdrType_initLong(&sample->camera_type)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->cols)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->rows)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->step1)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->type1)) {
        return RTI_FALSE;
    }                
            
    DDS_CharSeq_initialize(&sample->imageData1);
                
    if (!DDS_CharSeq_set_maximum(&sample->imageData1,
            (1228800))) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_initLong(&sample->step2)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->type2)) {
        return RTI_FALSE;
    }                
            
    DDS_CharSeq_initialize(&sample->imageData2);
                
    if (!DDS_CharSeq_set_maximum(&sample->imageData2,
            (1228800))) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}

void dds_image_message_t_finalize(
    dds_image_message_t* sample)
{
    dds_image_message_t_finalize_ex(sample,RTI_TRUE);
}
        
void dds_image_message_t_finalize_ex(
    dds_image_message_t* sample,RTIBool deletePointers)
{        

    DDS_CharSeq_finalize(&sample->imageData1);
            
    DDS_CharSeq_finalize(&sample->imageData2);
            
}

RTIBool dds_image_message_t_copy(
    dds_image_message_t* dst,
    const dds_image_message_t* src)
{        

    if (!RTICdrType_copyLong(
        &dst->camera_type, &src->camera_type)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->cols, &src->cols)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->rows, &src->rows)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->step1, &src->step1)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->type1, &src->type1)) {
        return RTI_FALSE;
    }
            
    if (!DDS_CharSeq_copy_no_alloc(&dst->imageData1,
                                          &src->imageData1)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->step2, &src->step2)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->type2, &src->type2)) {
        return RTI_FALSE;
    }
            
    if (!DDS_CharSeq_copy_no_alloc(&dst->imageData2,
                                          &src->imageData2)) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}


/**
 * <<IMPLEMENTATION>>
 *
 * Defines:  TSeq, T
 *
 * Configure and implement 'dds_image_message_t' sequence class.
 */
#define T dds_image_message_t
#define TSeq dds_image_message_tSeq
#define T_initialize_ex dds_image_message_t_initialize_ex
#define T_finalize_ex   dds_image_message_t_finalize_ex
#define T_copy       dds_image_message_t_copy

#ifndef NDDS_STANDALONE_TYPE
#include "dds_c/generic/dds_c_sequence_TSeq.gen"
#ifdef __cplusplus
#include "dds_cpp/generic/dds_cpp_sequence_TSeq.gen"
#endif
#else
#include "dds_c_sequence_TSeq.gen"
#ifdef __cplusplus
#include "dds_cpp_sequence_TSeq.gen"
#endif
#endif

#undef T_copy
#undef T_finalize_ex
#undef T_initialize_ex
#undef TSeq
#undef T

