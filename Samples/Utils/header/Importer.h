/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_IMPORTER_H
#define MT_RRT_SAMPLE_IMPORTER_H

#include <map>
#include <set>
#include <string>
#include <vector>

namespace mt::sample {
    class Importer {
    public:
        /** @brief Reads and parse a textual file wher each row should have at least 2 values.
         * The first value in each row respresents the name of a field, while all the following should
         * be numbers to associate to that field.
         * The numbers parsed and related to a specific field can be accessed using find(...)
         *  @param the file storing the data to parse
         *  @param the fields that should be present in the file. Additional fields might be present but the passed ones should be mandatory
         *  present at least once in the file
         */
        template<typename ... Args>
        Importer(const std::string& fileName, Args ... requiredFields) 
            : requiredFields(parseRequired(requiredFields...)) {
            this->parseData(fileName);
        };

        /** @return the values associated to a field name (which might be repeated in the file)
         */
        std::vector<std::vector<float>*> find(const std::string& field);

    private:
        template<typename ... Args>
        static std::set<std::string> parseRequired(Args ... requiredFields) {
            std::set<std::string> fields;
            parseRequired(fields, requiredFields...);
            return fields;
        };
        template<typename ... Args, typename Str>
        static std::set<std::string>& parseRequired(std::set<std::string>& fields, Str name, Args ... requiredFields) {
            if(fields.find(name) != fields.end()) {
                throw 1;
            }
            fields.emplace(name);
            return parseRequired(fields, requiredFields...);
        };
        template<typename Str>
        static std::set<std::string>& parseRequired(std::set<std::string>& fields, Str name) {
            if(fields.find(name) != fields.end()) {
                throw 1;
            }
            fields.emplace(name);
            return fields;
        };
        static std::set<std::string>& parseRequired(std::set<std::string>& fields) {
            return fields;
        };

        void parseData(const std::string& fileName);

        const std::set<std::string> requiredFields;
        std::multimap<std::string, std::vector<float>> data;
    };
}

#endif