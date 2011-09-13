
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_obstacle_map_message_t.idl using "rtiddsgen".
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



#include "dds_obstacle_map_message_t.h"

/* ========================================================================= */
const char *dds_obstacle_map_message_tTYPENAME = "dds_obstacle_map_message_t";

DDS_TypeCode* dds_obstacle_map_message_t_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode dds_obstacle_map_message_t_g_tc_data_sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(8388608,NULL);

    static DDS_TypeCode_Member dds_obstacle_map_message_t_g_tc_members[11]=
    {
        {
            (char *)"utime",/* Member name */
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
            (char *)"type",/* Member name */
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
            (char *)"resolution",/* Member name */
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
            (char *)"num_rows",/* Member name */
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
            (char *)"num_cols",/* Member name */
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
            (char *)"map_r0",/* Member name */
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
            (char *)"map_c0",/* Member name */
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
            (char *)"array_r0",/* Member name */
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
            (char *)"array_c0",/* Member name */
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
            (char *)"length",/* Member name */
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
            (char *)"data",/* Member name */
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

    static DDS_TypeCode dds_obstacle_map_message_t_g_tc =
    {{
        DDS_TK_STRUCT,/* Kind */
        DDS_BOOLEAN_FALSE, /* Ignored */
        -1,/* Ignored */
        (char *)"dds_obstacle_map_message_t", /* Name */
        NULL, /* Ignored */
        0, /* Ignored */
        0, /* Ignored */
        NULL, /* Ignored */
        11, /* Number of members */
        dds_obstacle_map_message_t_g_tc_members, /* Members */
        DDS_VM_NONE /* Ignored */
    }}; /* Type code for dds_obstacle_map_message_t*/

    if (is_initialized) {
        return &dds_obstacle_map_message_t_g_tc;
    }

    dds_obstacle_map_message_t_g_tc_data_sequence._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;

    dds_obstacle_map_message_t_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_longlong;
    dds_obstacle_map_message_t_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_obstacle_map_message_t_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_float;
    dds_obstacle_map_message_t_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[6]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[7]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[8]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_long;
    dds_obstacle_map_message_t_g_tc_members[9]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_longlong;
    dds_obstacle_map_message_t_g_tc_members[10]._representation._typeCode = (RTICdrTypeCode *)&dds_obstacle_map_message_t_g_tc_data_sequence;

    is_initialized = RTI_TRUE;

    return &dds_obstacle_map_message_t_g_tc;
}


RTIBool dds_obstacle_map_message_t_initialize(
    dds_obstacle_map_message_t* sample) {
  return dds_obstacle_map_message_t_initialize_ex(sample,RTI_TRUE);
}
        
RTIBool dds_obstacle_map_message_t_initialize_ex(
    dds_obstacle_map_message_t* sample,RTIBool allocatePointers)
{

    void* buffer;                
    buffer = NULL;        

    if (!RTICdrType_initLongLong(&sample->utime)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->type)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initFloat(&sample->resolution)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->num_rows)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->num_cols)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->map_r0)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->map_c0)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->array_r0)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLong(&sample->array_c0)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initLongLong(&sample->length)) {
        return RTI_FALSE;
    }                
            
    DDS_CharSeq_initialize(&sample->data);
                
    if (!DDS_CharSeq_set_maximum(&sample->data,
            (8388608))) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}

void dds_obstacle_map_message_t_finalize(
    dds_obstacle_map_message_t* sample)
{
    dds_obstacle_map_message_t_finalize_ex(sample,RTI_TRUE);
}
        
void dds_obstacle_map_message_t_finalize_ex(
    dds_obstacle_map_message_t* sample,RTIBool deletePointers)
{        

    DDS_CharSeq_finalize(&sample->data);
            
}

RTIBool dds_obstacle_map_message_t_copy(
    dds_obstacle_map_message_t* dst,
    const dds_obstacle_map_message_t* src)
{        

    if (!RTICdrType_copyLongLong(
        &dst->utime, &src->utime)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->type, &src->type)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyFloat(
        &dst->resolution, &src->resolution)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->num_rows, &src->num_rows)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->num_cols, &src->num_cols)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->map_r0, &src->map_r0)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->map_c0, &src->map_c0)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->array_r0, &src->array_r0)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLong(
        &dst->array_c0, &src->array_c0)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyLongLong(
        &dst->length, &src->length)) {
        return RTI_FALSE;
    }
            
    if (!DDS_CharSeq_copy_no_alloc(&dst->data,
                                          &src->data)) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}


/**
 * <<IMPLEMENTATION>>
 *
 * Defines:  TSeq, T
 *
 * Configure and implement 'dds_obstacle_map_message_t' sequence class.
 */
#define T dds_obstacle_map_message_t
#define TSeq dds_obstacle_map_message_tSeq
#define T_initialize_ex dds_obstacle_map_message_t_initialize_ex
#define T_finalize_ex   dds_obstacle_map_message_t_finalize_ex
#define T_copy       dds_obstacle_map_message_t_copy

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

