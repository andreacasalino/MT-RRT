/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/../../src/Extender.h>
#include <MT-RRT-core/Planner.h>

#include <nlohmann/json.hpp>

namespace mt_rrt::utils {
void from_file(nlohmann::json &j, const std::string &fileName);

void to_json(nlohmann::json &j, const Tree &subject);

void to_json(nlohmann::json &j, const std::vector<Tree> &subject);

class Converter {
public:
  Converter() = default;
  virtual ~Converter() = default;

  virtual void toJson(nlohmann::json &recipient,
                      const Connector &connector) const = 0;

  virtual void toJson(nlohmann::json &recipient, const std::vector<State> &sol,
                      const Connector &connector) const {
    recipient = sol;
  }
};

template <typename ConnectorT> class ConverterT : public Converter {
public:
  virtual std::shared_ptr<ProblemDescription>
  fromJson(const std::optional<Seed> &seed,
           const nlohmann::json &content) const = 0;

  void toJson(nlohmann::json &recipient,
              const Connector &connector) const final {
    const auto *ptr = dynamic_cast<const ConnectorT *>(&connector);
    if (ptr == nullptr) {
      throw Error{"Not a valid connector to convert"};
    }
    toJson_(recipient, *ptr);
  };

protected:
  virtual void toJson_(nlohmann::json &recipient,
                       const ConnectorT &connector) const = 0;
};

void to_json(nlohmann::json &j, const ProblemDescription &problem,
             const PlannerSolution &solution, const Converter &converter);

void to_json(nlohmann::json &j, const Extender &subject,
             const Converter &converter);
} // namespace mt_rrt::utils
