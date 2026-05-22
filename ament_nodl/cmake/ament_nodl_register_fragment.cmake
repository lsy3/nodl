# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Register a NoDL fragment in the ament resource index.
#
# Publishes the contents of a NoDL file under the ``nodl_fragments`` resource type.
# The resource key is ``<package>__<name>``.
# NoDL documents reference the fragment by ``nodl://<package>/<name>``.
#
# Consumers retrieve the content via::
#
#   ament_index_python.packages.get_resource('nodl_fragments', '<pkg>__<name>')
#
# The source file is also installed under ``share/<package>/nodl/fragments/`` for direct filesystem access.
#
# Example::
#
#   ament_nodl_register_fragment(tf_listener
#     FILE nodl/tf_listener.nodl.yaml
#   )
#
# :param fragment_name: name of the fragment.
#   Combined with PACKAGE to form the resource key and the ``nodl://<package>/<name>`` URI.
# :type fragment_name: string
# :param FILE: path to the NoDL file containing the fragment definition.
#   May be absolute or relative to ``CMAKE_CURRENT_SOURCE_DIR``.
# :type FILE: string
# :param PACKAGE: package name to use in the resource key.
#   Defaults to ``${PROJECT_NAME}``.
# :type PACKAGE: string
#
# @public
#
function(ament_nodl_register_fragment fragment_name)
  cmake_parse_arguments(_ANF "" "FILE;PACKAGE" "" ${ARGN})

  if(NOT _ANF_FILE)
    message(FATAL_ERROR "ament_nodl_register_fragment: FILE is required")
  endif()
  if(NOT _ANF_PACKAGE)
    set(_ANF_PACKAGE "${PROJECT_NAME}")
  endif()

  get_filename_component(_abs_file "${_ANF_FILE}" ABSOLUTE
    BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

  if(NOT EXISTS "${_abs_file}")
    message(WARNING
      "ament_nodl_register_fragment: file not found at configure time: ${_abs_file}")
  endif()

  # Validate the fragment at build time.
  # --fragment additionally rejects base/fragments at the top level since
  # nested composition is intentionally disallowed in v2.
  set(_stamp_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_nodl/nodl_fragments")
  set(_stamp "${_stamp_dir}/${_ANF_PACKAGE}__${fragment_name}.valid.stamp")
  file(MAKE_DIRECTORY "${_stamp_dir}")
  add_custom_command(
    OUTPUT "${_stamp}"
    DEPENDS "${_abs_file}"
    COMMAND "${Python3_EXECUTABLE}" -m nodl_schema --fragment "${_abs_file}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${_stamp}"
    COMMENT "Validating NoDL fragment ${_ANF_PACKAGE}/${fragment_name}"
    VERBATIM
  )
  add_custom_target(_ament_nodl_validate_fragment_${_ANF_PACKAGE}__${fragment_name} ALL
    DEPENDS "${_stamp}"
  )

  # Both installs reference the source file directly so they pick up its
  # current bytes at install time, not a snapshot from configure time.
  install(
    FILES "${_abs_file}"
    DESTINATION "share/ament_index/resource_index/nodl_fragments"
    RENAME "${_ANF_PACKAGE}__${fragment_name}")

  install(
    FILES "${_abs_file}"
    DESTINATION "share/${_ANF_PACKAGE}/nodl/fragments")
endfunction()
