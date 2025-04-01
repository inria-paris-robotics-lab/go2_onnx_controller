#define MCAP_IMPLEMENTATION
#include "writer.hpp"

#include "BuildFileDescriptorSet.hpp"
#include "ObservationAction.pb.h"
#include "spdlog/spdlog.h"

Writer::Writer() {
  std::string filename = std::tmpnam(nullptr);
  filename += ".mcap";
  
  // Open the .mcap file to write to
  {
    auto options = mcap::McapWriterOptions("");
    const auto res = writer_.open(filename, options);
    if (!res.ok()) {
      throw std::runtime_error("Failed to open mcap file:" + filename + " !"); 
    }
  }

  // Set up the channel
  mcap::ChannelId channelId;
  {
   mcap::Schema schema(
      "ObservationAction", "protobuf",
      foxglove::BuildFileDescriptorSet(ObservationAction::descriptor()).SerializeAsString());
   
   // Register the schema 
   writer_.addSchema(schema);  
  }
}
