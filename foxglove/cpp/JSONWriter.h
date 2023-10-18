#pragma once
#include "rust/cxx.h"
#include <memory>

namespace foxglove {

class JSONWriter {
    public:
        JSONWriter();
        ~JSONWriter();

        void write();
    
    private:
};

std::unique_ptr<JSONWriter> new_json_writer();


} // namespace foxglove
