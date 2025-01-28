#include "symbol.h"

namespace gtsam {

std::unique_ptr<Symbol> default_symbol() { return std::make_unique<Symbol>(); }

std::unique_ptr<Symbol> new_symbol(unsigned char character, uint64_t index) {
    return std::make_unique<Symbol>(character, index);
}

} // namespace gtsam