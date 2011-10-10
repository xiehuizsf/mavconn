
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_mavlink_message_t.idl using "rtiddsgen".
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



#include "dds_mavlink_message_t.h"

/* ========================================================================= */
const char *dds_mavlink_message_tTYPENAME = "dds_mavlink_message_t";

DDS_TypeCode* dds_mavlink_message_t_get_typecode()
{
    static RTIBool is_initialized = RTI_FALSE;

    static DDS_TypeCode dds_mavlink_message_t_g_tc_payload64_array = DDS_INITIALIZE_ARRAY_TYPECODE(1,33,NULL,NULL);

    static DDS_TypeCode_Member dds_mavlink_message_t_g_tc_members[8]=
    {
        {
            (char *)"checksum",/* Member name */
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
            (char *)"magic",/* Member name */
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
            (char *)"len",/* Member name */
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
            (char *)"seq",/* Member name */
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
            (char *)"sysid",/* Member name */
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
            (char *)"compid",/* Member name */
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
            (char *)"msgid",/* Member name */
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
            (char *)"payload64",/* Member name */
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

    static DDS_TypeCode dds_mavlink_message_t_g_tc =
    {{
        DDS_TK_STRUCT,/* Kind */
        DDS_BOOLEAN_FALSE, /* Ignored */
        -1,/* Ignored */
        (char *)"dds_mavlink_message_t", /* Name */
        NULL, /* Ignored */
        0, /* Ignored */
        0, /* Ignored */
        NULL, /* Ignored */
        8, /* Number of members */
        dds_mavlink_message_t_g_tc_members, /* Members */
        DDS_VM_NONE /* Ignored */
    }}; /* Type code for dds_mavlink_message_t*/

    if (is_initialized) {
        return &dds_mavlink_message_t_g_tc;
    }

    dds_mavlink_message_t_g_tc_payload64_array._data._typeCode = (RTICdrTypeCode *)&DDS_g_tc_longlong;

    dds_mavlink_message_t_g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_short;
    dds_mavlink_message_t_g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[3]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[4]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[5]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[6]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_char;
    dds_mavlink_message_t_g_tc_members[7]._representation._typeCode = (RTICdrTypeCode *)&dds_mavlink_message_t_g_tc_payload64_array;

    is_initialized = RTI_TRUE;

    return &dds_mavlink_message_t_g_tc;
}


RTIBool dds_mavlink_message_t_initialize(
    dds_mavlink_message_t* sample) {
  return dds_mavlink_message_t_initialize_ex(sample,RTI_TRUE);
}
        
RTIBool dds_mavlink_message_t_initialize_ex(
    dds_mavlink_message_t* sample,RTIBool allocatePointers)
{

    if (!RTICdrType_initShort(&sample->checksum)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->magic)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->len)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->seq)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->sysid)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->compid)) {
        return RTI_FALSE;
    }                
            
    if (!RTICdrType_initChar(&sample->msgid)) {
        return RTI_FALSE;
    }                
                
    if (!RTICdrType_initArray(
        sample->payload64, (33), RTI_CDR_LONG_LONG_SIZE)) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}

void dds_mavlink_message_t_finalize(
    dds_mavlink_message_t* sample)
{
    dds_mavlink_message_t_finalize_ex(sample,RTI_TRUE);
}
        
void dds_mavlink_message_t_finalize_ex(
    dds_mavlink_message_t* sample,RTIBool deletePointers)
{        

}

RTIBool dds_mavlink_message_t_copy(
    dds_mavlink_message_t* dst,
    const dds_mavlink_message_t* src)
{        

    if (!RTICdrType_copyShort(
        &dst->checksum, &src->checksum)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->magic, &src->magic)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->len, &src->len)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->seq, &src->seq)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->sysid, &src->sysid)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->compid, &src->compid)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyChar(
        &dst->msgid, &src->msgid)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrType_copyArray(
        dst->payload64, src->payload64, (33), RTI_CDR_LONG_LONG_SIZE)) {
        return RTI_FALSE;
    }
            

    return RTI_TRUE;
}


/**
 * <<IMPLEMENTATION>>
 *
 * Defines:  TSeq, T
 *
 * Configure and implement 'dds_mavlink_message_t' sequence class.
 */
#define T dds_mavlink_message_t
#define TSeq dds_mavlink_message_tSeq
#define T_initialize_ex dds_mavlink_message_t_initialize_ex
#define T_finalize_ex   dds_mavlink_message_t_finalize_ex
#define T_copy       dds_mavlink_message_t_copy

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

