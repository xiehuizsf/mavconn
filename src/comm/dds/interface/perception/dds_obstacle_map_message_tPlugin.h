
/*
  WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

  This file was generated from dds_obstacle_map_message_t.idl using "rtiddsgen".
  The rtiddsgen tool is part of the RTI Data Distribution Service distribution.
  For more information, type 'rtiddsgen -help' at a command shell
  or consult the RTI Data Distribution Service manual.
*/

#ifndef dds_obstacle_map_message_tPlugin_1684969828_h
#define dds_obstacle_map_message_tPlugin_1684969828_h

#include "dds_obstacle_map_message_t.h"




struct RTICdrStream;

#ifndef pres_typePlugin_h
#include "pres/pres_typePlugin.h"
#endif


#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, start exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport __declspec(dllexport)
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define dds_obstacle_map_message_tPlugin_get_sample PRESTypePluginDefaultEndpointData_getSample 
#define dds_obstacle_map_message_tPlugin_return_sample PRESTypePluginDefaultEndpointData_returnSample 
#define dds_obstacle_map_message_tPlugin_get_buffer PRESTypePluginDefaultEndpointData_getBuffer 
#define dds_obstacle_map_message_tPlugin_return_buffer PRESTypePluginDefaultEndpointData_returnBuffer 
 

#define dds_obstacle_map_message_tPlugin_create_sample PRESTypePluginDefaultEndpointData_createSample 
#define dds_obstacle_map_message_tPlugin_destroy_sample PRESTypePluginDefaultEndpointData_deleteSample 

/* --------------------------------------------------------------------------------------
    Support functions:
 * -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern dds_obstacle_map_message_t*
dds_obstacle_map_message_tPluginSupport_create_data_ex(RTIBool allocate_pointers);

NDDSUSERDllExport extern dds_obstacle_map_message_t*
dds_obstacle_map_message_tPluginSupport_create_data(void);

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPluginSupport_copy_data(
    dds_obstacle_map_message_t *out,
    const dds_obstacle_map_message_t *in);

NDDSUSERDllExport extern void 
dds_obstacle_map_message_tPluginSupport_destroy_data_ex(
    dds_obstacle_map_message_t *sample,RTIBool deallocate_pointers);

NDDSUSERDllExport extern void 
dds_obstacle_map_message_tPluginSupport_destroy_data(
    dds_obstacle_map_message_t *sample);

NDDSUSERDllExport extern void 
dds_obstacle_map_message_tPluginSupport_print_data(
    const dds_obstacle_map_message_t *sample,
    const char *desc,
    unsigned int indent);

 

/* ----------------------------------------------------------------------------
    Callback functions:
 * ---------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginParticipantData 
dds_obstacle_map_message_tPlugin_on_participant_attached(
    void *registration_data, 
    const struct PRESTypePluginParticipantInfo *participant_info,
    RTIBool top_level_registration, 
    void *container_plugin_context,
    RTICdrTypeCode *typeCode);

NDDSUSERDllExport extern void 
dds_obstacle_map_message_tPlugin_on_participant_detached(
    PRESTypePluginParticipantData participant_data);
    
NDDSUSERDllExport extern PRESTypePluginEndpointData 
dds_obstacle_map_message_tPlugin_on_endpoint_attached(
    PRESTypePluginParticipantData participant_data,
    const struct PRESTypePluginEndpointInfo *endpoint_info,
    RTIBool top_level_registration, 
    void *container_plugin_context);

NDDSUSERDllExport extern void 
dds_obstacle_map_message_tPlugin_on_endpoint_detached(
    PRESTypePluginEndpointData endpoint_data);

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_copy_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t *out,
    const dds_obstacle_map_message_t *in);

/* --------------------------------------------------------------------------------------
    (De)Serialize functions:
 * -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_serialize(
    PRESTypePluginEndpointData endpoint_data,
    const dds_obstacle_map_message_t *sample,
    struct RTICdrStream *stream, 
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_deserialize_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t *sample, 
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);

 
NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_deserialize(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t **sample, 
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_sample, 
    void *endpoint_plugin_qos);



NDDSUSERDllExport extern RTIBool
dds_obstacle_map_message_tPlugin_skip(
    PRESTypePluginEndpointData endpoint_data,
    struct RTICdrStream *stream, 
    RTIBool skip_encapsulation,  
    RTIBool skip_sample, 
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern unsigned int 
dds_obstacle_map_message_tPlugin_get_serialized_sample_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int size);

NDDSUSERDllExport extern unsigned int 
dds_obstacle_map_message_tPlugin_get_serialized_sample_min_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int size);

NDDSUSERDllExport extern unsigned int
dds_obstacle_map_message_tPlugin_get_serialized_sample_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment,
    const dds_obstacle_map_message_t * sample);


/* --------------------------------------------------------------------------------------
    Key Management functions:
 * -------------------------------------------------------------------------------------- */

NDDSUSERDllExport extern PRESTypePluginKeyKind 
dds_obstacle_map_message_tPlugin_get_key_kind(void);

NDDSUSERDllExport extern unsigned int 
dds_obstacle_map_message_tPlugin_get_serialized_key_max_size(
    PRESTypePluginEndpointData endpoint_data,
    RTIBool include_encapsulation,
    RTIEncapsulationId encapsulation_id,
    unsigned int current_alignment);

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_serialize_key(
    PRESTypePluginEndpointData endpoint_data,
    const dds_obstacle_map_message_t *sample,
    struct RTICdrStream *stream,
    RTIBool serialize_encapsulation,
    RTIEncapsulationId encapsulation_id,
    RTIBool serialize_key,
    void *endpoint_plugin_qos);

NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_deserialize_key_sample(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t * sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);

 
NDDSUSERDllExport extern RTIBool 
dds_obstacle_map_message_tPlugin_deserialize_key(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t ** sample,
    RTIBool * drop_sample,
    struct RTICdrStream *stream,
    RTIBool deserialize_encapsulation,
    RTIBool deserialize_key,
    void *endpoint_plugin_qos);


NDDSUSERDllExport extern RTIBool
dds_obstacle_map_message_tPlugin_serialized_sample_to_key(
    PRESTypePluginEndpointData endpoint_data,
    dds_obstacle_map_message_t *sample,
    struct RTICdrStream *stream, 
    RTIBool deserialize_encapsulation,  
    RTIBool deserialize_key, 
    void *endpoint_plugin_qos);

     
/* Plugin Functions */
NDDSUSERDllExport extern struct PRESTypePlugin*
dds_obstacle_map_message_tPlugin_new(void);

NDDSUSERDllExport extern void
dds_obstacle_map_message_tPlugin_delete(struct PRESTypePlugin *);
 

#if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
/* If the code is building on Windows, stop exporting symbols.
*/
#undef NDDSUSERDllExport
#define NDDSUSERDllExport
#endif


#ifdef __cplusplus
}
#endif
        

#endif /* dds_obstacle_map_message_tPlugin_1684969828_h */
