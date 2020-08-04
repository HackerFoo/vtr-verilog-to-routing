#include <vector>
#include <utility>
#include <cstddef>
#include <set>
#include <random>
#include <algorithm>

#include "catch.hpp"

#include "globals.h"
#include "rr_graph.h"

// Class for building a group of connected edges.
class EdgeGroups {
  public:
    EdgeGroups()
        : node_count_(std::numeric_limits<size_t>::max()) {}

    // set_node_count must be invoked before create_sets with the count of
    // nodes. Assumption is that from_node/to_node arguments to
    // add_non_config_edge are less than node_count.
    void set_node_count(size_t node_count) {
        node_count_ = node_count;
    }

    // Adds non-configurable edge to be group.
    //
    // Returns true if this is a new edge.
    bool add_non_config_edge(int from_node, int to_node) {
        auto result = node_edges_.insert(std::make_pair(from_node, to_node));
        return result.second;
    }

    // After add_non_config_edge has been called for all edges, create_sets
    // will form groups of nodes that are connected via non-configurable
    // edges.
    void create_sets() {
        rr_non_config_node_sets_map_.clear();

        size_t num_groups = 0;
        std::vector<std::pair<int, int>> merges;

        VTR_ASSERT(node_count_ != std::numeric_limits<size_t>::max());
        std::vector<int> node_to_node_set(node_count_, OPEN);

        // First nievely make node groups.  When an edge joins two groups,
        // mark it for cleanup latter.
        for (const auto& edge : node_edges_) {
            VTR_ASSERT(edge.first >= 0 && static_cast<size_t>(edge.first) < node_count_);
            VTR_ASSERT(edge.second >= 0 && static_cast<size_t>(edge.second) < node_count_);

            int& from_set = node_to_node_set[edge.first];
            int& to_set = node_to_node_set[edge.second];

            if (from_set == OPEN && to_set == OPEN) {
                from_set = num_groups++;
                to_set = from_set;
            } else if (from_set == OPEN && to_set != OPEN) {
                from_set = to_set;
            } else if (from_set != OPEN && to_set == OPEN) {
                to_set = from_set;
            } else {
                VTR_ASSERT(from_set != OPEN);
                VTR_ASSERT(to_set != OPEN);

                if (from_set != to_set) {
                    merges.push_back(std::make_pair(
                        std::min(from_set, to_set),
                        std::max(from_set, to_set)));
                }
            }
        }

        // We are going to always collapse sets to lower ids, so sort
        // the merge list to ensure that the merge first elements are always
        // increasing.
        std::sort(merges.begin(), merges.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.first < b.first;
        });

        // Update final_set_map with the final merge id for the second element.
        // The first element will either be the final value, or already have
        // an entry in the final_set_map (because sorting), so we can depend on
        // find_target_set(first) returning a stable value.
        std::unordered_map<int, int> final_set_map;
        for (const auto& merge : merges) {
            VTR_ASSERT(merge.first < merge.second);
            VTR_ASSERT(merge.first != OPEN);
            VTR_ASSERT(merge.second != OPEN);

            int target_set = find_target_set(final_set_map, merge.first);

            final_set_map.insert(std::make_pair(merge.second, target_set));
        }

        // Finalize merges between node set ids.
        for (auto& set : node_to_node_set) {
            set = find_target_set(final_set_map, set);
        }
        final_set_map.clear();

        // Sanity check the node sets.
        for (const auto& edge : node_edges_) {
            VTR_ASSERT(node_to_node_set[edge.first] != OPEN);
            VTR_ASSERT(node_to_node_set[edge.second] != OPEN);
            VTR_ASSERT(node_to_node_set[edge.first] == node_to_node_set[edge.second]);
        }

        // Create compact set of sets.
        for (size_t inode = 0; inode < node_to_node_set.size(); ++inode) {
            if (node_to_node_set[inode] != OPEN) {
                rr_non_config_node_sets_map_[node_to_node_set[inode]].push_back(inode);
            }
        }
    }

    // Create t_non_configurable_rr_sets from set data.
    t_non_configurable_rr_sets output_sets() {
        t_non_configurable_rr_sets sets;
        for (auto& item : rr_non_config_node_sets_map_) {
            std::set<t_node_edge> edge_set;
            std::set<int> node_set(item.second.begin(), item.second.end());

            for (const auto& edge : node_edges_) {
                if (node_set.find(edge.first) != node_set.end()) {
                    edge_set.emplace(t_node_edge(edge.first, edge.second));
                }
            }

            sets.node_sets.emplace(std::move(node_set));
            sets.edge_sets.emplace(std::move(edge_set));
        }

        return sets;
    }

    // Set device context structures for non-configurable node sets.
    void set_device_context() {
        std::vector<std::vector<int>> rr_non_config_node_sets;
        for (auto& item : rr_non_config_node_sets_map_) {
            rr_non_config_node_sets.emplace_back(std::move(item.second));
        }

        std::unordered_map<int, int> rr_node_to_non_config_node_set;
        for (size_t set = 0; set < rr_non_config_node_sets.size(); ++set) {
            for (const auto inode : rr_non_config_node_sets[set]) {
                rr_node_to_non_config_node_set.insert(
                    std::make_pair(inode, set));
            }
        }

        auto& device_ctx = g_vpr_ctx.mutable_device();
        device_ctx.rr_non_config_node_sets = std::move(rr_non_config_node_sets);
        device_ctx.rr_node_to_non_config_node_set = std::move(rr_node_to_non_config_node_set);
    }

  private:
    // Final target set for given set id.
    static int find_target_set(
        const std::unordered_map<int, int>& final_set_map,
        int set) {
        int target_set = set;
        while (true) {
            auto iter = final_set_map.find(target_set);
            if (iter != final_set_map.end()) {
                target_set = iter->second;
            } else {
                break;
            }
        }

        return target_set;
    }

    // Number of nodes.  All elements of node_edges_ should be less than this
    // value.
    size_t node_count_;

    // Set of non-configurable edges.
    std::set<std::pair<int, int>> node_edges_;

    // Compact set of node sets. Map key is arbitrary.
    std::map<int, std::vector<int>> rr_non_config_node_sets_map_;
};

namespace {

TEST_CASE("edge_groups_create_sets", "[vpr]") {
    std::vector<std::pair<int, int>> edges;
    std::vector<std::set<int>> connected_sets{{{1, 2, 3, 4, 5, 6, 7, 8},
                                               {9, 0}}};
    int max_node_id = 0;
    for (auto set : connected_sets) {
        int last = *set.cbegin();
        std::for_each(std::next(set.cbegin()),
                      set.cend(),
                      [&](int node) {
                          edges.push_back(std::make_pair(last, node));
                          last = node;
                          max_node_id = std::max(max_node_id, node);
                      });
    }
    std::vector<int> nodes(max_node_id + 1);
    std::iota(nodes.begin(), nodes.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());

    for (int i = 0; i < 100; i++) {
        auto random_edges = edges;
        auto random_connected_sets = connected_sets;
        auto random_nodes = nodes;
        std::shuffle(random_nodes.begin(), random_nodes.end(), g);
        for (auto& edge : random_edges) {
            edge.first = random_nodes[edge.first];
            edge.second = random_nodes[edge.second];
        }
        std::shuffle(random_edges.begin(), random_edges.end(), g);
        EdgeGroups groups;
        groups.set_node_count(random_nodes.size());
        for (auto edge : random_edges) {
            groups.add_non_config_edge(edge.first, edge.second);
        }
        groups.create_sets();
        t_non_configurable_rr_sets sets = groups.output_sets();
        for (auto set : connected_sets) {
            std::set<int> random_set;
            for (auto elem : set) {
                random_set.insert(random_nodes[elem]);
            }
            REQUIRE(sets.node_sets.find(random_set) != sets.node_sets.end());
        }
    }
}

} // namespace
