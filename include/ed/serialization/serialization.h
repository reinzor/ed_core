#ifndef ED_SERIALIZATION_H_
#define ED_SERIALIZATION_H_

#include "ed/types.h"

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

namespace tue {
namespace config {
class Reader;
class Writer;
}
}

namespace ed
{

// SERIALIZATION

void serialize(const WorldModel& wm, tue::config::Writer& w);

// DESERIALIZATION

void deserialize(tue::config::Reader& r, UpdateRequest& req);

}

#endif
