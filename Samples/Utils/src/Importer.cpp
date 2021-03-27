/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Importer.h>
#include <sstream>
#include <list>
#include <fstream>
#include <Error.h>

namespace mt::sample {
    std::list<std::string> trimmer(const std::string& line) {
        std::istringstream iss(line);
        std::list<std::string> slices;
        while (!iss.eof()) {
            slices.emplace_back(std::string());
            iss >> slices.back();
            if(slices.back().empty()) slices.pop_back();
        }
        return slices;
    };

    inline float convert(const std::string& buff) {
        return static_cast<float>(std::atof(buff.c_str()));
    }

    std::vector<const std::vector<float>*> Importer::find(const std::string& field) const {
        auto range = this->data.equal_range(field);
        std::vector<const std::vector<float>*> res;
        res.reserve(std::distance(range.first, range.second));
        for(auto it = range.first; it!=range.second; ++it) {
            res.push_back(&it->second);
        }
        return res;
    }

    void Importer::parseData(const std::string& fileName) {
        std::ifstream f(fileName);
        if(!f.is_open()) {
            throw Error("invaldid file to import");
        }

        std::string line;
        std::list<std::string> slices;
        std::vector<float> values;
        std::size_t lineCounter = 0;
        while (!f.eof()) {
            std::getline(f, line);
            slices = trimmer(line);
            if(slices.empty()) continue;
            if(slices.size() < 2) {
                throw Error(std::string("foudn invalid content at line") + std::to_string(lineCounter));
            }
            
            values.resize(slices.size() - 1);
            std::size_t pos = slices.size() - 2;
            while(slices.size() != 1) {
                values[pos] = convert(slices.back());
                slices.pop_back();
                --pos;
            }

            this->data.emplace(slices.front() , values);
            ++lineCounter;
        }

        // check all required inputs were found
        for(auto it = this->requiredFields.begin(); it!=this->requiredFields.end(); ++it) {
            if(this->find(*it).empty()) {
                throw Error(*it + std::string("wes not found"));
            }
        }
    }
}