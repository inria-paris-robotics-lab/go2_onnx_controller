#include "mcap/writer.hpp"

class Writer {
  public:
    Writer();

  private:
    mcap::McapWriter writer_;
};
