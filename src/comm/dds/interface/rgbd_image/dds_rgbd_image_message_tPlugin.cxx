
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_rgbd_image_message_t.idl using "rtiddsgen".
  The rtiddsgen tool is part of the RTI Data Distribution Service distribution.
  For more information, type 'rtiddsgen -help' at a command shell
  or consult the RTI Data Distribution Service manual.
*/


#include <string.h>

#ifdef __cplusplus
#ifndef ndds_cpp_h
  #include "ndds/ndds_cpp.h"
#endif
#else
#ifndef ndds_c_h
  #include "ndds/ndds_c.h"
#endif
#endif

#ifndef osapi_type_h
  #include "osapi/osapi_type.h"
#endif
#ifndef osapi_heap_h
  #include "osapi/osapi_heap.h"
#endif

#ifndef osapi_utility_h
  #include "osapi/osapi_utility.h"
#endif

#ifndef cdr_type_h
  #include "cdr/cdr_type.h"
#endif

#ifndef cdr_encapsulation_h
  #include "cdr/cdr_encapsulation.h"
#endif

#ifndef cdr_stream_h
  #include "cdr/cdr_stream.h"
#endif

#ifndef pres_typePlugin_h
  #include "pres/pres_typePlugin.h"
#endif



#include "dds_rgbd_image_message_tPlugin.h"


/* --------------------------------------------------------------------------------------
 *  Type dds_rgbd_image_message_t
 * -------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------
    Support functions:
 * -------------------------------------------------------------------------------------- */


dds_rgbd_image_message_t *
dds_rgbd_image_message_tPluginSupport_create_data_ex(RTIBool allocate_pointers){
    dds_rgbd_image_message_t *sample = NULL;

    RTIOsapiHeap_allocateStructure(
        &sample, dds_rgbd_image_message_t);

    if(sample != NULL) {
        if (!dds_rgbd_image_message_t_initialize_ex(sample,allocate_pointers)) {
            RTIOsapiHeap_freeStructure(&sample);
            return NULL;
        }
    }
    return sample; 
}


dds_rgbd_image_message_t *
dds_rgbd_image_message_tPluginSupport_create_data(void)
{
    return dds_rgbd_image_message_tPluginSupport_create_data_ex(RTI_TRUE);
}


void 
dds_rgbd_image_message_tPluginSupport_destroy_data_ex(
    dds_rgbd_image_message_t *sample,RTIBool deallocate_pointers) {

    dds_rgbd_image_message_t_finalize_ex(sample,deallocate_pointers);

    RTIOsapiHeap_freeStructure(sample);
}


void 
dds_rgbd_image_message_tPluginSupport_destroy_data(
    dds_rgbd_image_message_t *sample) {

    dds_rgbd_image_message_tPluginSupport_destroy_data_ex(sample,RTI_TRUE);

}


RTIBool 
dds_rgbd_image_message_tPluginSupport_copy_data(
    dds_rgbd_image_message_t *dst,
    const dds_rgbd_image_message_t *src)
{
    return dds_rgbd_image_message_t_copy(dst,src);
}


void 
dds_rgbd_image_message_tPluginSupport_print_data(
    const dds_rgbd_image_message_t *sample,
    const char *desc,
    unsigned int indent_level)
{


    RTICdrType_printIndent(indent_level);

    if (desc != NULL) {
      RTILog_debug("%s:\n", desc);
    } else {
      RTILog_debug("\n");
    }

    if (sample == NULL) {
      RTILog_debug("NULL\n");
      return;
    }


    RTICdrType_printLong(
        &sample->camera_config, "camera_config", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->camera_type, "camera_type", indent_level + 1);
            
    RTICdrType_printUnsignedLongLong(
        &sample->timestamp, "timestamp", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->roll, "roll", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->pitch, "pitch", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->yaw, "yaw", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->lon, "lon", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->lat, "lat", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->alt, "alt", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->ground_x, "ground_x", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->ground_y, "ground_y", indent_level + 1);
            
    RTICdrType_printFloat(
        &sample->ground_z, "ground_z", indent_level + 1);
            
    RTICdrType_printArray(
        sample->camera_matrix, (9), RTI_CDR_FLOAT_SIZE,
        (RTICdrTypePrintFunction)RTICdrType_printFloat,
        "camera_matrix", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->cols, "cols", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->rows, "rows", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->step1, "step1", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->type1, "type1", indent_level + 1);
            
    if (&sample->imageData1 == NULL) {
        RTICdrType_printIndent(indent_level+1);
        RTILog_debug("imageData1: NULL\n");    
    } else {
    
        if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1) != NULL) {
            RTICdrType_printArray(
                DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1),
                DDS_CharSeq_get_length(&sample->imageData1),
                RTI_CDR_CHAR_SIZE,
                (RTICdrTypePrintFunction)RTICdrType_printChar,
                "imageData1", indent_level + 1);
        } else {
            RTICdrType_printPointerArray(
                DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData1),
                DDS_CharSeq_get_length(&sample->imageData1),
               (RTICdrTypePrintFunction)RTICdrType_printChar,
               "imageData1", indent_level + 1);
        }
    
    }
            
    RTICdrType_printLong(
        &sample->step2, "step2", indent_level + 1);
            
    RTICdrType_printLong(
        &sample->type2, "type2", indent_level + 1);
            
    if (&sample->imageData2 == NULL) {
        RTICdrType_printIndent(indent_level+1);
        RTILog_debug("imageData2: NULL\n");    
    } else {
    
        if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2) != NULL) {
            RTICdrType_printArray(
                DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2),
                DDS_CharSeq_get_length(&sample->imageData2),
                RTI_CDR_CHAR_SIZE,
                (RTICdrTypePrintFunction)RTICdrType_printChar,
                "imageData2", indent_level + 1);
        } else {
            RTICdrType_printPointerArray(
                DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData2),
                DDS_CharSeq_get_length(&sample->imageData2),
               (RTICdrTypePrintFunction)RTICdrType_printChar,
               "imageData2", indent_level + 1);
        }
    
    }
            

}



/* ----------------------------------------------------------------------------
    Callback functions:
 * ---------------------------------------------------------------------------- */



PRESTypePluginParticipantData 
dds_rgbd_image_message_tPlugin_on_participant_attached(
    void *registration_data,
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration,
    void *container_plugin_context,
    RTICdrTypeCode *type_code)
{

    return PRESTypePluginDefaultParticipantData_new(participant_info);

}


void 
dds_rgbd_image_message_tPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data)
{

  PRESTypePluginDefaultParticipantData_delete(participant_data);
}


PRESTypePluginEndpointData
dds_rgbd_image_message_tPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *containerPluginContext)
{
    PRESTypePluginEndpointData epd = NULL;

    epd = PRESTypePluginDefaultEndpointData_new(
            participant_data,
            endpoint_info,
            (PRESTypePluginDefaultEndpointDataCreateSampleFunction)
            dds_rgbd_image_message_tPluginSupport_create_data,
            (PRESTypePluginDefaultEndpointDataDestroySampleFunction)
            dds_rgbd_image_message_tPluginSupport_destroy_data,
            NULL, NULL);

    if (epd == NULL) {
        return NULL;
    }

    
    if (endpoint_info->endpointKind == PRES_TYPEPLUGIN_ENDPOINT_WRITER) {
        if (PRESTypePluginDefaultEndpointData_createWriterPool(
                epd,
                endpoint_info,
            (PRESTypePluginGetSerializedSampleMaxSizeFunction)
                dds_rgbd_image_message_tPlugin_get_serialized_sample_max_size, epd,
            (PRESTypePluginGetSerializedSampleSizeFunction)
            dds_rgbd_image_message_tPlugin_get_serialized_sample_size,
            epd) == RTI_FALSE) {
            PRESTypePluginDefaultEndpointData_delete(epd);
            return NULL;
        }
    }
    


    return epd;    
}


void 
dds_rgbd_image_message_tPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data)
{  

    PRESTypePluginDefaultEndpointData_delete(endpoint_data);
}


RTIBool 
dds_rgbd_image_message_tPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t *dst,
    const dds_rgbd_image_message_t *src)
{
    return dds_rgbd_image_message_tPluginSupport_copy_data(dst,src);
}

/* --------------------------------------------------------------------------------------
    (De)Serialize functions:
 * -------------------------------------------------------------------------------------- */


RTIBool 
dds_rgbd_image_message_tPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const dds_rgbd_image_message_t *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;


  if(serialize_encapsulation) {

    if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream, encapsulation_id)) {
        return RTI_FALSE;
    }

    position = RTICdrStream_resetAlignment(stream);

  }


  if(serialize_sample) {

    if (!RTICdrStream_serializeLong(
        stream, &sample->camera_config)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->camera_type)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeUnsignedLongLong(
        stream, &sample->timestamp)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->roll)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->pitch)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->yaw)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->lon)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->lat)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->alt)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->ground_x)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->ground_y)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeFloat(
        stream, &sample->ground_z)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializePrimitiveArray(
        stream, (void*)sample->camera_matrix, (9), RTI_CDR_FLOAT_TYPE)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->cols)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->rows)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->step1)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->type1)) {
        return RTI_FALSE;
    }
            
    if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1) != NULL) {
        if (!RTICdrStream_serializePrimitiveSequence(
            stream,
            DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1),
            DDS_CharSeq_get_length(&sample->imageData1),
            (3891200),
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    } else {
        if (!RTICdrStream_serializePrimitivePointerSequence(
            stream,
            (const void **)DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData1),
            DDS_CharSeq_get_length(&sample->imageData1),
            (3891200),
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->step2)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_serializeLong(
        stream, &sample->type2)) {
        return RTI_FALSE;
    }
            
    if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2) != NULL) {
        if (!RTICdrStream_serializePrimitiveSequence(
            stream,
            DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2),
            DDS_CharSeq_get_length(&sample->imageData2),
            (3891200),
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    } else {
        if (!RTICdrStream_serializePrimitivePointerSequence(
            stream,
            (const void **)DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData2),
            DDS_CharSeq_get_length(&sample->imageData2),
            (3891200),
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    }
            
  }


  if(serialize_encapsulation) {
    RTICdrStream_restoreAlignment(stream,position);
  }


  return RTI_TRUE;
}


RTIBool 
dds_rgbd_image_message_tPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t *sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;


    if(deserialize_encapsulation) {
        /* Deserialize encapsulation */

        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);

    }
    

    if(deserialize_sample) {

    if (!RTICdrStream_deserializeLong(
        stream, &sample->camera_config)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->camera_type)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeUnsignedLongLong(
        stream, &sample->timestamp)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->roll)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->pitch)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->yaw)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->lon)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->lat)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->alt)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->ground_x)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->ground_y)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeFloat(
        stream, &sample->ground_z)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializePrimitiveArray(
        stream, (void*)sample->camera_matrix, (9), RTI_CDR_FLOAT_TYPE)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->cols)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->rows)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->step1)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->type1)) {
        return RTI_FALSE;
    }
            
    {
        RTICdrUnsignedLong sequence_length;

        if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1) != NULL) {
            if (!RTICdrStream_deserializePrimitiveSequence(
                stream,
                DDS_CharSeq_get_contiguous_bufferI(&sample->imageData1),
                &sequence_length,
                DDS_CharSeq_get_maximum(&sample->imageData1),
                RTI_CDR_CHAR_TYPE)) {
                return RTI_FALSE;
            }
        } else {
            if (!RTICdrStream_deserializePrimitivePointerSequence(
                stream,
                (void **)DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData1),
                &sequence_length,
                DDS_CharSeq_get_maximum(&sample->imageData1),
                RTI_CDR_CHAR_TYPE)) {
                return RTI_FALSE;
            }
        }
        if (!DDS_CharSeq_set_length(&sample->imageData1, sequence_length)) {
            return RTI_FALSE;
        }
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->step2)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_deserializeLong(
        stream, &sample->type2)) {
        return RTI_FALSE;
    }
            
    {
        RTICdrUnsignedLong sequence_length;

        if (DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2) != NULL) {
            if (!RTICdrStream_deserializePrimitiveSequence(
                stream,
                DDS_CharSeq_get_contiguous_bufferI(&sample->imageData2),
                &sequence_length,
                DDS_CharSeq_get_maximum(&sample->imageData2),
                RTI_CDR_CHAR_TYPE)) {
                return RTI_FALSE;
            }
        } else {
            if (!RTICdrStream_deserializePrimitivePointerSequence(
                stream,
                (void **)DDS_CharSeq_get_discontiguous_bufferI(&sample->imageData2),
                &sequence_length,
                DDS_CharSeq_get_maximum(&sample->imageData2),
                RTI_CDR_CHAR_TYPE)) {
                return RTI_FALSE;
            }
        }
        if (!DDS_CharSeq_set_length(&sample->imageData2, sequence_length)) {
            return RTI_FALSE;
        }
    }
            
    }


    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }


    return RTI_TRUE;
}

 
 
RTIBool 
dds_rgbd_image_message_tPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t **sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,   
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos)
{

    return dds_rgbd_image_message_tPlugin_deserialize_sample( 
        endpoint_data, (sample != NULL)?*sample:NULL,
        stream, deserialize_encapsulation, deserialize_sample, 
        endpoint_plugin_qos);
 
}



RTIBool dds_rgbd_image_message_tPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream,   
    RTIBool skip_encapsulation,
    RTIBool skip_sample, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;


    if(skip_encapsulation) {
        if (!RTICdrStream_skipEncapsulation(stream)) {
            return RTI_FALSE;
        }


        position = RTICdrStream_resetAlignment(stream);

    }

    if (skip_sample) {

    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipUnsignedLongLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipFloat(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipPrimitiveArray(
        stream, (9), RTI_CDR_FLOAT_TYPE)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    {
        RTICdrUnsignedLong sequence_length;

        if (!RTICdrStream_skipPrimitiveSequence(
            stream,
            &sequence_length,
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    if (!RTICdrStream_skipLong(stream)) {
        return RTI_FALSE;
    }
            
    {
        RTICdrUnsignedLong sequence_length;

        if (!RTICdrStream_skipPrimitiveSequence(
            stream,
            &sequence_length,
            RTI_CDR_CHAR_TYPE)) {
            return RTI_FALSE;
        }
    }
            
    }


    if(skip_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }


    return RTI_TRUE;
}


unsigned int 
dds_rgbd_image_message_tPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;


    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }

        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;

    }


    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveArrayMaxSizeSerialized(
        current_alignment, (9), RTI_CDR_FLOAT_TYPE);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment, (3891200), RTI_CDR_CHAR_TYPE);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment, (3891200), RTI_CDR_CHAR_TYPE);
            
    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }

    return current_alignment - initial_alignment;
}


unsigned int 
dds_rgbd_image_message_tPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;


    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }

        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;

    }


    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveArrayMaxSizeSerialized(
        current_alignment, (9), RTI_CDR_FLOAT_TYPE);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment, 0, RTI_CDR_CHAR_TYPE);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment +=  RTICdrType_getPrimitiveSequenceMaxSizeSerialized(
        current_alignment, 0, RTI_CDR_CHAR_TYPE);
            
    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }

    return current_alignment - initial_alignment;
}


/* Returns the size of the sample in its serialized form (in bytes).
 * It can also be an estimation in excess of the real buffer needed 
 * during a call to the serialize() function.
 * The value reported does not have to include the space for the
 * encapsulation flags.
 */
unsigned int
dds_rgbd_image_message_tPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const dds_rgbd_image_message_t * sample) 
{

    unsigned int initial_alignment = current_alignment;

    unsigned int encapsulation_size = current_alignment;


    if (include_encapsulation) {

        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }

        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;

    }


    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getFloatMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getPrimitiveArrayMaxSizeSerialized(
        current_alignment, (9), RTI_CDR_FLOAT_TYPE);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getPrimitiveSequenceSerializedSize(
        current_alignment, 
        DDS_CharSeq_get_length(&sample->imageData1),
        RTI_CDR_CHAR_TYPE);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getLongMaxSizeSerialized(
        current_alignment);
            
    current_alignment += RTICdrType_getPrimitiveSequenceSerializedSize(
        current_alignment, 
        DDS_CharSeq_get_length(&sample->imageData2),
        RTI_CDR_CHAR_TYPE);
            
    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }

    return current_alignment - initial_alignment;
}

/* --------------------------------------------------------------------------------------
    Key Management functions:
 * -------------------------------------------------------------------------------------- */


PRESTypePluginKeyKind 
dds_rgbd_image_message_tPlugin_get_key_kind(void)
{

    return PRES_TYPEPLUGIN_NO_KEY;
     
}


RTIBool 
dds_rgbd_image_message_tPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const dds_rgbd_image_message_t *sample, 
    struct RTICdrStream *stream,    
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;


    if(serialize_encapsulation) {
        if (!RTICdrStream_serializeAndSetCdrEncapsulation(stream, encapsulation_id)) {
            return RTI_FALSE;
        }


        position = RTICdrStream_resetAlignment(stream);

    }

    if(serialize_key) {

        if (!dds_rgbd_image_message_tPlugin_serialize(
                endpoint_data,
                sample,
                stream,
                RTI_FALSE, encapsulation_id,
                RTI_TRUE,
                endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    
    }


    if(serialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }


    return RTI_TRUE;
}


RTIBool dds_rgbd_image_message_tPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    char * position = NULL;


    if(deserialize_encapsulation) {
        /* Deserialize encapsulation */
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;  
        }


        position = RTICdrStream_resetAlignment(stream);

    }

    if (deserialize_key) {

        if (!dds_rgbd_image_message_tPlugin_deserialize_sample(
                endpoint_data, sample, stream,
                RTI_FALSE, RTI_TRUE, 
                endpoint_plugin_qos)) {
            return RTI_FALSE;
        }
    
    }


    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }


    return RTI_TRUE;
}


 
RTIBool dds_rgbd_image_message_tPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos)
{
    return dds_rgbd_image_message_tPlugin_deserialize_key_sample(
        endpoint_data, (sample != NULL)?*sample:NULL, stream,
        deserialize_encapsulation, deserialize_key, endpoint_plugin_qos);
}



unsigned int
dds_rgbd_image_message_tPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment)
{

    unsigned int encapsulation_size = current_alignment;


    unsigned int initial_alignment = current_alignment;

    if (include_encapsulation) {
        if (!RTICdrEncapsulation_validEncapsulationId(encapsulation_id)) {
            return 1;
        }


        RTICdrStream_getEncapsulationSize(encapsulation_size);
        encapsulation_size -= current_alignment;
        current_alignment = 0;
        initial_alignment = 0;

    }
        

    current_alignment += dds_rgbd_image_message_tPlugin_get_serialized_sample_max_size(
        endpoint_data,RTI_FALSE, encapsulation_id, current_alignment);
    
    if (include_encapsulation) {
        current_alignment += encapsulation_size;
    }

    return current_alignment - initial_alignment;
}


RTIBool 
dds_rgbd_image_message_tPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    dds_rgbd_image_message_t *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos)
{
    char * position = NULL;


    if(deserialize_encapsulation) {
        if (!RTICdrStream_deserializeAndSetCdrEncapsulation(stream)) {
            return RTI_FALSE;
        }

        position = RTICdrStream_resetAlignment(stream);

    }

    if (deserialize_key) {

        if (!dds_rgbd_image_message_tPlugin_deserialize_sample(
            endpoint_data, sample, stream, RTI_FALSE, 
            RTI_TRUE, endpoint_plugin_qos)) {
            return RTI_FALSE;
        }

    }


    if(deserialize_encapsulation) {
        RTICdrStream_restoreAlignment(stream,position);
    }


    return RTI_TRUE;
}


 

/* ------------------------------------------------------------------------
 * Plug-in Installation Methods
 * ------------------------------------------------------------------------ */
 
struct PRESTypePlugin *dds_rgbd_image_message_tPlugin_new(void) 
{ 
    struct PRESTypePlugin *plugin = NULL;
    const struct PRESTypePluginVersion PLUGIN_VERSION = 
        PRES_TYPE_PLUGIN_VERSION_2_0;

    RTIOsapiHeap_allocateStructure(
        &plugin, struct PRESTypePlugin);
    if (plugin == NULL) {
       return NULL;
    }

    plugin->version = PLUGIN_VERSION;

    /* set up parent's function pointers */
    plugin->onParticipantAttached =
        (PRESTypePluginOnParticipantAttachedCallback)
        dds_rgbd_image_message_tPlugin_on_participant_attached;
    plugin->onParticipantDetached =
        (PRESTypePluginOnParticipantDetachedCallback)
        dds_rgbd_image_message_tPlugin_on_participant_detached;
    plugin->onEndpointAttached =
        (PRESTypePluginOnEndpointAttachedCallback)
        dds_rgbd_image_message_tPlugin_on_endpoint_attached;
    plugin->onEndpointDetached =
        (PRESTypePluginOnEndpointDetachedCallback)
        dds_rgbd_image_message_tPlugin_on_endpoint_detached;

    plugin->copySampleFnc =
        (PRESTypePluginCopySampleFunction)
        dds_rgbd_image_message_tPlugin_copy_sample;
    plugin->createSampleFnc =
        (PRESTypePluginCreateSampleFunction)
        dds_rgbd_image_message_tPlugin_create_sample;
    plugin->destroySampleFnc =
        (PRESTypePluginDestroySampleFunction)
        dds_rgbd_image_message_tPlugin_destroy_sample;

    plugin->serializeFnc =
        (PRESTypePluginSerializeFunction)
        dds_rgbd_image_message_tPlugin_serialize;
    plugin->deserializeFnc =
        (PRESTypePluginDeserializeFunction)
        dds_rgbd_image_message_tPlugin_deserialize;
    plugin->getSerializedSampleMaxSizeFnc =
        (PRESTypePluginGetSerializedSampleMaxSizeFunction)
        dds_rgbd_image_message_tPlugin_get_serialized_sample_max_size;
    plugin->getSerializedSampleMinSizeFnc =
        (PRESTypePluginGetSerializedSampleMinSizeFunction)
        dds_rgbd_image_message_tPlugin_get_serialized_sample_min_size;

    plugin->getSampleFnc =
        (PRESTypePluginGetSampleFunction)
        dds_rgbd_image_message_tPlugin_get_sample;
    plugin->returnSampleFnc =
        (PRESTypePluginReturnSampleFunction)
        dds_rgbd_image_message_tPlugin_return_sample;

    plugin->getKeyKindFnc =
        (PRESTypePluginGetKeyKindFunction)
        dds_rgbd_image_message_tPlugin_get_key_kind;

 
    /* These functions are only used for keyed types. As this is not a keyed
    type they are all set to NULL
    */
    plugin->serializeKeyFnc = NULL;
    plugin->deserializeKeyFnc = NULL;
    plugin->getKeyFnc = NULL;
    plugin->returnKeyFnc = NULL;
    plugin->instanceToKeyFnc = NULL;
    plugin->keyToInstanceFnc = NULL;
    plugin->getSerializedKeyMaxSizeFnc = NULL;
    plugin->instanceToKeyHashFnc = NULL;
    plugin->serializedSampleToKeyHashFnc = NULL;
    plugin->serializedKeyToKeyHashFnc = NULL;
    
    plugin->typeCode =  (struct RTICdrTypeCode *)dds_rgbd_image_message_t_get_typecode();
    
    plugin->languageKind = PRES_TYPEPLUGIN_DDS_TYPE; 

    /* Serialized buffer */
    plugin->getBuffer = 
        (PRESTypePluginGetBufferFunction)
        dds_rgbd_image_message_tPlugin_get_buffer;
    plugin->returnBuffer = 
        (PRESTypePluginReturnBufferFunction)
        dds_rgbd_image_message_tPlugin_return_buffer;
    plugin->getSerializedSampleSizeFnc =
        (PRESTypePluginGetSerializedSampleSizeFunction)
        dds_rgbd_image_message_tPlugin_get_serialized_sample_size;

    plugin->endpointTypeName = dds_rgbd_image_message_tTYPENAME;

    return plugin;
}

void
dds_rgbd_image_message_tPlugin_delete(struct PRESTypePlugin *plugin)
{
    RTIOsapiHeap_freeStructure(plugin);
} 
