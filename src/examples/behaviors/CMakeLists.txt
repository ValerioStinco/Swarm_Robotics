if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)

###########
  add_executable(kilobot_v1 kilobot_v1.c)
  target_link_libraries(kilobot_v1 argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_v2 kilobot_v2.c)
  target_link_libraries(kilobot_v2 argos3plugin_simulator_kilolib)
###########
  add_executable(move_to_position move_to_position.c)  
  target_link_libraries(move_to_position argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_ALF_client-server_server kilobot_ALF_client-server_server.c)  
  target_link_libraries(kilobot_ALF_client-server_server argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_ALF_client-server_client kilobot_ALF_client-server_client.c)  
  target_link_libraries(kilobot_ALF_client-server_client argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_ALF_dhtf kilobot_ALF_dhtf.c)  
  target_link_libraries(kilobot_ALF_dhtf argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_ALF_dhtf2 kilobot_ALF_dhtf2.c)  
  target_link_libraries(kilobot_ALF_dhtf2 argos3plugin_simulator_kilolib)
###########
  add_executable(kilobot_ALF_cres kilobot_ALF_cres.c)  
  target_link_libraries(kilobot_ALF_cres argos3plugin_simulator_kilolib)

  add_executable(kilobot_ALF_crec kilobot_ALF_crec.c)  
  target_link_libraries(kilobot_ALF_crec argos3plugin_simulator_kilolib)
###########

  #
  # Lab0: Blinky
  #
  add_executable(blinky blinky.c)
  target_link_libraries(blinky argos3plugin_simulator_kilolib)

  #
  # Lab1.2: Simple Movement
  #
  add_executable(simple_movement simple_movement.c)
  target_link_libraries(simple_movement argos3plugin_simulator_kilolib)

  #
  # Lab1.3: Non-blocked Movement
  #
  add_executable(nonblocked_movement nonblocked_movement.c)
  target_link_libraries(nonblocked_movement argos3plugin_simulator_kilolib)

  #
  # Lab2.1-2.2: Test Speaker and Test Listener
  #
  add_executable(test_speaker test_speaker.c)
  target_link_libraries(test_speaker argos3plugin_simulator_kilolib)
  add_executable(test_listener test_listener.c)
  target_link_libraries(test_listener argos3plugin_simulator_kilolib)

  #
  # Lab2.3-2.4: Modified Test Speaker and Test Listener
  #
  add_executable(test_speaker_mod test_speaker_mod.c)
  target_link_libraries(test_speaker_mod argos3plugin_simulator_kilolib)
  add_executable(test_listener_mod test_listener_mod.c)
  target_link_libraries(test_listener_mod argos3plugin_simulator_kilolib)

  #
  # Lab3: Disperse
  #
  add_executable(disperse disperse.c)
  target_link_libraries(disperse argos3plugin_simulator_kilolib)

  #
  # Lab4: Orbit
  #
  add_executable(orbit_star orbit_star.c)
  target_link_libraries(orbit_star argos3plugin_simulator_kilolib)
  add_executable(orbit_planet orbit_planet.c)
  target_link_libraries(orbit_planet argos3plugin_simulator_kilolib)

  #
  # Lab5: Move to Light
  #
  add_executable(move_to_light move_to_light.c)
  target_link_libraries(move_to_light argos3plugin_simulator_kilolib)

  #
  # Lab6: Simple Gradient
  #
  add_executable(gradient_simple gradient_simple.c)
  target_link_libraries(gradient_simple argos3plugin_simulator_kilolib)

  #
  # Lab7: Sync
  #
  add_executable(sync sync.c)
  target_link_libraries(sync argos3plugin_simulator_kilolib)


  #
  # ARK loop function: demoC
  #
  add_executable(forager forager.c)  
  target_link_libraries(forager argos3plugin_simulator_kilolib)

  #
  # ARK loop function: clustering
  #
  add_executable(clustering clustering.c)  
  target_link_libraries(clustering argos3plugin_simulator_kilolib)

  #
  # Debugging example
  #
  add_executable(test_debug test_debug.h test_debug.c)
  target_link_libraries(test_debug argos3plugin_simulator_kilolib)
  endif(ARGOS_BUILD_FOR_SIMULATOR)
