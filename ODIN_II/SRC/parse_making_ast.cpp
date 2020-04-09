/*
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <sstream>
#include "odin_globals.h"
#include "odin_types.h"
#include "odin_util.h"
#include "ast_util.h"
#include "parse_making_ast.h"
#include "string_cache.h"
#include "verilog_bison_user_defined.h"
#include "verilog_bison.h"
#include "hard_blocks.h"
#include "vtr_util.h"
#include "vtr_memory.h"
#include "vtr_path.h"

#include "scope_util.h"

//for module
STRING_CACHE* modules_inputs_sc;
STRING_CACHE* modules_outputs_sc;
STRING_CACHE* module_instances_sc;

//for function
STRING_CACHE* functions_inputs_sc;
STRING_CACHE* functions_outputs_sc;

STRING_CACHE* tasks_inputs_sc;
STRING_CACHE* tasks_outputs_sc;

STRING_CACHE* module_names_to_idx;
STRING_CACHE* instantiated_modules;

ast_node_t** module_variables_not_defined;
int size_module_variables_not_defined;
ast_node_t** function_instantiations_instance;
int size_function_instantiations;
ast_node_t** function_instantiations_instance_by_module;
int size_function_instantiations_by_module;

ast_node_t** task_instantiations_instance;
int size_task_instantiations;
ast_node_t** task_instantiations_instance_by_module;
int size_task_instantiations_by_module;

ast_t* verilog_ast;
long num_modules;

int num_functions;
ast_node_t** ast_functions;

long num_tasks;
ast_node_t** ast_tasks;

ast_node_t** all_file_items_list;
int size_all_file_items_list;

/*
 * File-scope function declarations
 */
ast_node_t* newFunctionAssigning(ast_node_t* expression1, ast_node_t* expression2, int line_number);
ast_node_t* resolve_ports(ids top_type, ast_node_t* symbol_list);
bool is_valid_identifier(char* str);

/*---------------------------------------------------------------------------------------------
 * (function: parse_to_ast)
 *-------------------------------------------------------------------------------------------*/
void parse_to_ast() {
    extern void push_include(const char* file_name);

    /* read all the files in the configuration file */
    current_parse_file = configuration.list_of_file_names.size() - 1;
    int parse_counter = current_parse_file;

    while (parse_counter >= 0) {
        push_include(configuration.list_of_file_names[parse_counter].c_str());
        parse_counter--;
    }

    current_parse_file = 0;

    /* parse all the files */
    yyparse();

    /* verify that the parser did not trigger delayed errors */
    verify_delayed_error();

    /* for error messages - this is in case we use any of the parser functions after parsing (i.e. create_case_control_signals()) */
    current_parse_file = -1;
}

/*---------------------------------------------------------------------------------------------
 * (function: init_parser)
 *-------------------------------------------------------------------------------------------*/
ast_t* init_parser() {
    ast_t* ast = allocate_ast();

    /* create string caches to hookup PORTS with INPUT and OUTPUTs.  This is made per module and will be cleaned and remade at next_module */
    modules_inputs_sc = sc_new_string_cache();
    modules_outputs_sc = sc_new_string_cache();
    functions_inputs_sc = sc_new_string_cache();
    functions_outputs_sc = sc_new_string_cache();
    tasks_inputs_sc = sc_new_string_cache();
    tasks_outputs_sc = sc_new_string_cache();
    instantiated_modules = sc_new_string_cache();
    module_instances_sc = sc_new_string_cache();
    /* record of each of the individual modules */
    num_modules = 0; // we're going to record all the modules in a list so we can build a tree of them later
    num_functions = 0;
    num_tasks = 0;
    ast_functions = NULL;
    ast_tasks = NULL;
    function_instantiations_instance = NULL;
    module_variables_not_defined = NULL;
    size_module_variables_not_defined = 0;
    size_function_instantiations = 0;
    function_instantiations_instance_by_module = NULL;
    size_function_instantiations_by_module = 0;
    size_task_instantiations = 0;
    size_task_instantiations_by_module = 0;

    /* keeps track of all the ast roots */
    all_file_items_list = NULL;
    size_all_file_items_list = 0;

    return ast;
}

/*---------------------------------------------------------------------------------------------
 * (function: cleanup_parser)
 *-------------------------------------------------------------------------------------------*/
void cleanup_parser() {
    modules_inputs_sc = sc_free_string_cache(modules_inputs_sc);
    modules_outputs_sc = sc_free_string_cache(modules_outputs_sc);
    functions_inputs_sc = sc_free_string_cache(functions_inputs_sc);
    functions_outputs_sc = sc_free_string_cache(functions_outputs_sc);
    tasks_inputs_sc = sc_free_string_cache(tasks_inputs_sc);
    tasks_outputs_sc = sc_free_string_cache(tasks_outputs_sc);
    instantiated_modules = sc_free_string_cache(instantiated_modules);
    module_instances_sc = sc_free_string_cache(module_instances_sc);
}

/*---------------------------------------------------------------------------------------------
 * (function: next_parsed_verilog_file)
 *-------------------------------------------------------------------------------------------*/
void next_parsed_verilog_file(ast_node_t* file_items_list) {
    long i;
    /* optimization entry point */
    printf("Optimizing module by AST based optimizations\n");
    if (configuration.output_ast_graphs == 1) {
        /* IF - we want outputs for the graphViz files of each module */
        for (i = 0; i < file_items_list->num_children; i++) {
            assert(file_items_list->children[i]);
            graphVizOutputAst(configuration.debug_output_path, file_items_list->children[i]);
        }
    }

    /* store the root of this files ast */
    all_file_items_list = (ast_node_t**)vtr::realloc(all_file_items_list, sizeof(ast_node_t*) * (size_all_file_items_list + 1));
    all_file_items_list[size_all_file_items_list] = file_items_list;
    size_all_file_items_list++;
}

/*---------------------------------------------------------------------------------------------
 * (function: newSymbolNode)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newSymbolNode(char* id, int line_number) {
    return create_tree_node_id(id, line_number, current_parse_file);
}

/*---------------------------------------------------------------------------------------------
 * (function: newNumberNode)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newNumberNode(char* num, int line_number) {
    ast_node_t* current_node = create_tree_node_number(num, line_number, current_parse_file);
    vtr::free(num);
    return current_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newList)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newList(ids node_type, ast_node_t* child, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(node_type, line_number, current_parse_file);
    /* allocate child nodes to this node */
    if (child) allocate_children_to_node(new_node, {child});

    return new_node;
}

ast_node_t* newfunctionList(ids node_type, ast_node_t* child, int line_number) {
    /* create a output node for this array reference that is going to be the first child */
    ast_node_t* output_node = create_node_w_type(IDENTIFIERS, line_number, current_parse_file);
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(node_type, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {output_node, child});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newList_entry)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newList_entry(ast_node_t* list, ast_node_t* child) {
    /* allocate child nodes to this node */
    if (child) add_child_to_node(list, child);

    return list;
}

/*---------------------------------------------------------------------------------------------
 * (function: newListReplicate)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newListReplicate(ast_node_t* exp, ast_node_t* child, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(REPLICATE, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {exp, child});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newBlock)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newBlock(char* id, ast_node_t* block_node) {
    oassert(block_node->type == BLOCK);
    if (id) {
        block_node->types.identifier = id;
        block_node->types.scope = pop_scope();
    } else {
        merge_top_scopes();
    }

    return block_node;
}

static ast_node_t* resolve_symbol_node(ast_node_t* symbol_node) {
    ast_node_t* to_return = NULL;

    switch (symbol_node->type) {
        case IDENTIFIERS: {
            ast_node_t* newNode = NULL;

            long sc_spot = sc_lookup_string(current_scope->param_sc, symbol_node->types.identifier);
            if (sc_spot != -1) {
                newNode = (ast_node_t*)current_scope->param_sc->data[sc_spot];
            }

            if (newNode && newNode->types.variable.is_parameter == true) {
                to_return = symbol_node;
            } else {
                error_message(PARSE_ERROR, symbol_node->line_number, current_parse_file,
                              "no match for parameter %s\n", symbol_node->types.identifier);
            }
            break;
        }
        case NUMBERS:
        case BINARY_OPERATION:
        case UNARY_OPERATION: {
            to_return = symbol_node;
            break;
        }
        default: {
            to_return = NULL;
            break;
        }
    }

    return to_return;
}

ast_node_t* resolve_ports(ids top_type, ast_node_t* symbol_list) {
    ast_node_t* unprocessed_ports = NULL;
    if (symbol_list) {
        for (long i = 0; i < symbol_list->num_children; i++) {
            if (symbol_list->children[i]->types.variable.is_port) {
                ast_node_t* this_port = symbol_list->children[i];

                if (!unprocessed_ports) {
                    unprocessed_ports = newList(VAR_DECLARE_LIST, this_port, this_port->line_number);
                } else {
                    unprocessed_ports = newList_entry(unprocessed_ports, this_port);
                }

                /* grab and update all typeless ports immediately following this one */
                long j = 0;
                for (j = i + 1; j < symbol_list->num_children && !(symbol_list->children[j]->types.variable.is_port); j++) {
                    /* port type */
                    symbol_list->children[j]->types.variable.is_input = this_port->types.variable.is_input;
                    symbol_list->children[j]->types.variable.is_output = this_port->types.variable.is_output;
                    symbol_list->children[j]->types.variable.is_inout = this_port->types.variable.is_inout;

                    /* net type */
                    symbol_list->children[j]->types.variable.is_wire = this_port->types.variable.is_wire;
                    symbol_list->children[j]->types.variable.is_reg = this_port->types.variable.is_reg;
                    symbol_list->children[j]->types.variable.is_integer = this_port->types.variable.is_integer;

                    /* signedness */
                    symbol_list->children[j]->types.variable.is_signed = this_port->types.variable.is_signed;

                    /* range */
                    if (symbol_list->children[j]->children[1] == NULL) {
                        symbol_list->children[j]->children[1] = ast_node_deep_copy(this_port->children[1]);
                        symbol_list->children[j]->children[2] = ast_node_deep_copy(this_port->children[2]);
                        symbol_list->children[j]->children[3] = ast_node_deep_copy(this_port->children[3]);
                        symbol_list->children[j]->children[4] = ast_node_deep_copy(this_port->children[4]);

                        if (this_port->num_children == 8) {
                            symbol_list->children[j]->children = (ast_node_t**)realloc(symbol_list->children[j]->children, sizeof(ast_node_t*) * 8);
                            symbol_list->children[j]->children[7] = symbol_list->children[j]->children[5];
                            symbol_list->children[j]->children[5] = ast_node_deep_copy(this_port->children[5]);
                            symbol_list->children[j]->children[6] = ast_node_deep_copy(this_port->children[6]);
                        }
                    }

                    /* error checking */
                    symbol_list->children[j] = markAndProcessPortWith(MODULE, NO_ID, NO_ID, symbol_list->children[j], this_port->types.variable.is_signed);
                }
            } else {
                long sc_spot = -1;

                if (top_type == MODULE) {
                    /* find the related INPUT or OUTPUT definition and store that instead */
                    if ((sc_spot = sc_lookup_string(modules_inputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)modules_inputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)modules_inputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_input);
                    } else if ((sc_spot = sc_lookup_string(modules_outputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)modules_outputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)modules_outputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_output);
                    } else {
                        error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file, "No matching declaration for port %s\n", symbol_list->children[i]->children[0]->types.identifier);
                    }
                } else if (top_type == FUNCTION) {
                    /* find the related INPUT or OUTPUT definition and store that instead */
                    if ((sc_spot = sc_lookup_string(functions_inputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)functions_inputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)functions_inputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_input);
                    } else if ((sc_spot = sc_lookup_string(functions_outputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)functions_outputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)functions_outputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_output);
                    } else {
                        error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file, "No matching declaration for port %s\n", symbol_list->children[i]->children[0]->types.identifier);
                    }
                } else if (top_type == TASK) {
                    /* find the related INPUT or OUTPUT definition and store that instead */
                    if ((sc_spot = sc_lookup_string(tasks_inputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)tasks_inputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)tasks_inputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_input);
                    } else if ((sc_spot = sc_lookup_string(tasks_outputs_sc, symbol_list->children[i]->children[0]->types.identifier)) != -1) {
                        oassert(((ast_node_t*)tasks_outputs_sc->data[sc_spot])->type == VAR_DECLARE);
                        free_whole_tree(symbol_list->children[i]);
                        symbol_list->children[i] = (ast_node_t*)tasks_outputs_sc->data[sc_spot];
                        oassert(symbol_list->children[i]->types.variable.is_output);
                    } else {
                        error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file, "No matching declaration for port %s\n", symbol_list->children[i]->children[0]->types.identifier);
                    }
                }
            }
            symbol_list->children[i]->types.variable.is_port = true;
        }
    }

    return unprocessed_ports;
}

ast_node_t* markAndProcessPortWith(ids top_type, ids port_id, ids net_id, ast_node_t* port, bool is_signed) {
    oassert((top_type == MODULE || top_type == FUNCTION || top_type == TASK)
            && "can only use MODULE, FUNCTION ot TASK as top type");

    long sc_spot;
    const char* top_type_name = NULL;
    STRING_CACHE* this_inputs_sc = NULL;
    STRING_CACHE* this_outputs_sc = NULL;

    if (top_type == MODULE) {
        top_type_name = "Module";
    } else if (top_type == FUNCTION) {
        top_type_name = "Function";
    } else {
        top_type_name = "Task";
    }

    ids temp_net_id = NO_ID;

    if (port->types.variable.is_port) {
        oassert(false && "Port was already marked");
    }

    if (top_type == MODULE) {
        this_inputs_sc = modules_inputs_sc;
        this_outputs_sc = modules_outputs_sc;

    } else if (top_type == FUNCTION) {
        this_inputs_sc = functions_inputs_sc;
        this_outputs_sc = functions_outputs_sc;
    } else if (top_type == TASK) {
        this_inputs_sc = tasks_inputs_sc;
        this_outputs_sc = tasks_outputs_sc;
    }

    /* look for processed inputs with this name */
    sc_spot = sc_lookup_string(this_inputs_sc, port->children[0]->types.identifier);
    if (sc_spot > -1 && ((ast_node_t*)this_inputs_sc->data[sc_spot])->types.variable.is_port) {
        error_message(PARSE_ERROR, port->line_number, current_parse_file, "%s already has input with this name %s\n",
                      top_type_name, ((ast_node_t*)this_inputs_sc->data[sc_spot])->children[0]->types.identifier);
    }

    /* look for processed outputs with this name */
    sc_spot = sc_lookup_string(this_outputs_sc, port->children[0]->types.identifier);
    if (sc_spot > -1 && ((ast_node_t*)this_outputs_sc->data[sc_spot])->types.variable.is_port) {
        error_message(PARSE_ERROR, port->line_number, current_parse_file, "%s already has output with this name %s\n",
                      top_type_name, ((ast_node_t*)this_outputs_sc->data[sc_spot])->children[0]->types.identifier);
    }

    switch (net_id) {
        case REG:
            if (port_id == INPUT) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Input cannot be defined as a reg\n");
            }
            if (port_id == INOUT) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Inout cannot be defined as a reg\n");
            }
            port->types.variable.is_reg = true;
            port->types.variable.is_wire = false;
            port->types.variable.is_integer = false;
            break;

        case INTEGER:
            if (port_id == INPUT) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Input cannot be defined as an integer\n");
            } else if (port_id == INOUT) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Inout cannot be defined as an integer\n");
            } else if (port_id == OUTPUT) {
                /* cannot support signed ports right now (integers are signed) */
                error_message(PARSE_ERROR, port->line_number, current_parse_file,
                              "Odin does not handle signed ports (%s)\n", port->children[0]->types.identifier);
            }
            port->types.variable.is_integer = true;
            port->types.variable.is_reg = false;
            port->types.variable.is_wire = false;
            break;

        case WIRE:
            if ((port->num_children == 6 && port->children[5] != NULL)
                || (port->num_children == 8 && port->children[7] != NULL)) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Ports of type net cannot be initialized\n");
            }
            port->types.variable.is_wire = true;
            port->types.variable.is_reg = false;
            port->types.variable.is_integer = false;
            break;

        default:
            if ((port->num_children == 6 && port->children[5] != NULL)
                || (port->num_children == 8 && port->children[7] != NULL)) {
                error_message(NETLIST_ERROR, port->line_number, port->file_number, "%s",
                              "Ports with undefined type cannot be initialized\n");
            }

            if (port->types.variable.is_reg
                && !(port->types.variable.is_wire)
                && !(port->types.variable.is_integer)) {
                temp_net_id = REG;
            } else if (port->types.variable.is_integer
                       && !(port->types.variable.is_wire)
                       && !(port->types.variable.is_reg)) {
                temp_net_id = INTEGER;
            } else if (port->types.variable.is_wire
                       && !(port->types.variable.is_reg)
                       && !(port->types.variable.is_integer)) {
                temp_net_id = WIRE;
            } else {
                /* port cannot have more than one type */
                oassert(!(port->types.variable.is_wire) && !(port->types.variable.is_reg) && !(port->types.variable.is_integer));
            }

            if (net_id == NO_ID) {
                /* will be marked later */
                net_id = temp_net_id;
            }

            break;
    }

    switch (port_id) {
        case INPUT:
            port->types.variable.is_input = true;
            port->types.variable.is_output = false;
            port->types.variable.is_inout = false;

            /* add this input to the modules string cache */
            sc_spot = sc_add_string(this_inputs_sc, port->children[0]->types.identifier);

            /* store the data which is an idx here */
            this_inputs_sc->data[sc_spot] = (void*)port;

            break;

        case OUTPUT:
            port->types.variable.is_output = true;
            port->types.variable.is_input = false;
            port->types.variable.is_inout = false;

            /* add this output to the modules string cache */
            sc_spot = sc_add_string(this_outputs_sc, port->children[0]->types.identifier);

            /* store the data which is an idx here */
            this_outputs_sc->data[sc_spot] = (void*)port;

            break;

        case INOUT:
            port->types.variable.is_inout = true;
            port->types.variable.is_input = false;
            port->types.variable.is_output = false;
            error_message(PARSE_ERROR, port->line_number, current_parse_file, "Odin does not handle inouts (%s)\n", port->children[0]->types.identifier);
            break;

        default:
            if (port->types.variable.is_input
                && !(port->types.variable.is_output)
                && !(port->types.variable.is_inout)) {
                port = markAndProcessPortWith(top_type, INPUT, net_id, port, is_signed);
            } else if (port->types.variable.is_output
                       && !(port->types.variable.is_input)
                       && !(port->types.variable.is_inout)) {
                port = markAndProcessPortWith(top_type, OUTPUT, net_id, port, is_signed);
            } else if (port->types.variable.is_inout
                       && !(port->types.variable.is_input)
                       && !(port->types.variable.is_output)) {
                error_message(PARSE_ERROR, port->line_number, current_parse_file, "Odin does not handle inouts (%s)\n", port->children[0]->types.identifier);
                port = markAndProcessPortWith(top_type, INOUT, net_id, port, is_signed);
            } else {
                // shouldn't ever get here...
                oassert(port->types.variable.is_input
                        || port->types.variable.is_output
                        || port->types.variable.is_inout);
            }

            break;
    }

    if (is_signed) {
        /* cannot support signed ports right now */
        error_message(PARSE_ERROR, port->line_number, current_parse_file,
                      "Odin does not handle signed ports (%s)\n", port->children[0]->types.identifier);
    }

    port->types.variable.is_signed = is_signed;
    port->types.variable.is_port = true;

    return port;
}

ast_node_t* markAndProcessParameterWith(ids id, ast_node_t* parameter, bool is_signed) {
    if (id == PARAMETER) {
        parameter->children[5]->types.variable.is_parameter = true;
        parameter->types.variable.is_parameter = true;
    } else if (id == LOCALPARAM) {
        parameter->children[5]->types.variable.is_localparam = true;
        parameter->types.variable.is_localparam = true;
    }

    if (is_signed) {
        /* cannot support signed parameters right now */
        error_message(PARSE_ERROR, parameter->line_number, current_parse_file,
                      "Odin does not handle signed parameters (%s)\n", parameter->children[0]->types.identifier);
    }

    parameter->children[5]->types.variable.is_signed = is_signed;
    parameter->types.variable.is_signed = is_signed;

    long sc_spot = -1;

    oassert(current_scope->param_sc);

    /* create an entry in the symbol table for this parameter */
    if ((sc_spot = sc_lookup_string(current_scope->param_sc, parameter->children[0]->types.identifier)) > -1) {
        error_message(PARSE_ERROR, parameter->children[5]->line_number, current_parse_file,
                      "Module already has parameter with this name (%s)\n", parameter->children[0]->types.identifier);
    }
    sc_spot = sc_add_string(current_scope->param_sc, parameter->children[0]->types.identifier);

    current_scope->param_sc->data[sc_spot] = (void*)parameter->children[5];

    return parameter;
}

ast_node_t* markAndProcessSymbolListWith(ids top_type, ids id, ast_node_t* symbol_list, bool is_signed) {
    long i;
    ast_node_t* range_min = NULL;
    ast_node_t* range_max = NULL;

    if (symbol_list) {
        if (symbol_list->children[0] && symbol_list->children[0]->children[1]) {
            range_max = resolve_symbol_node(symbol_list->children[0]->children[1]);
            range_min = resolve_symbol_node(symbol_list->children[0]->children[2]);
        }

        for (i = 0; i < symbol_list->num_children; i++) {
            if ((symbol_list->children[i]->children[1] == NULL) && (symbol_list->children[i]->children[2] == NULL)) {
                symbol_list->children[i]->children[1] = ast_node_deep_copy(range_max);
                symbol_list->children[i]->children[2] = ast_node_deep_copy(range_min);
            }

            if (top_type == MODULE || top_type == TASK) {
                switch (id) {
                    case PARAMETER:
                    case LOCALPARAM: {
                        markAndProcessParameterWith(id, symbol_list->children[i], is_signed);
                        break;
                    }
                    case INPUT:
                    case OUTPUT:
                    case INOUT:
                        symbol_list->children[i] = markAndProcessPortWith(top_type, id, NO_ID, symbol_list->children[i], is_signed);
                        break;
                    case WIRE:
                        if (is_signed) {
                            /* cannot support signed nets right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed nets (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_wire = true;
                        break;
                    case REG:
                        if (is_signed) {
                            /* cannot support signed regs right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed regs (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_reg = true;
                        break;
                    case INTEGER:
                        oassert(is_signed && "Integers must always be signed");
                        symbol_list->children[i]->types.variable.is_signed = is_signed;
                        symbol_list->children[i]->types.variable.is_integer = true;
                        break;
                    case GENVAR:
                        oassert(is_signed && "Genvars must always be signed");
                        symbol_list->children[i]->types.variable.is_signed = is_signed;
                        symbol_list->children[i]->types.variable.is_genvar = true;
                        break;
                    default:
                        oassert(false);
                }
            } else if (top_type == FUNCTION) {
                switch (id) {
                    case PARAMETER:
                    case LOCALPARAM: {
                        markAndProcessParameterWith(id, symbol_list->children[i], is_signed);
                        break;
                    }
                    case INPUT:
                    case OUTPUT:
                    case INOUT:
                        symbol_list->children[i] = markAndProcessPortWith(top_type, id, NO_ID, symbol_list->children[i], is_signed);
                        break;
                    case WIRE:
                        if ((symbol_list->children[i]->num_children == 6 && symbol_list->children[i]->children[5] != NULL)
                            || (symbol_list->children[i]->num_children == 8 && symbol_list->children[i]->children[7] != NULL)) {
                            error_message(NETLIST_ERROR, symbol_list->children[i]->line_number, symbol_list->children[i]->file_number, "%s",
                                          "Nets cannot be initialized\n");
                        }
                        if (is_signed) {
                            /* cannot support signed nets right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed nets (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_wire = true;
                        break;
                    case REG:
                        if (is_signed) {
                            /* cannot support signed regs right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed regs (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_reg = true;
                        break;
                    case INTEGER:
                        oassert(is_signed && "Integers must always be signed");
                        symbol_list->children[i]->types.variable.is_signed = is_signed;
                        symbol_list->children[i]->types.variable.is_integer = true;
                        break;
                    default:
                        oassert(false);
                }
            } else if (top_type == BLOCK) {
                switch (id) {
                    case LOCALPARAM: {
                        markAndProcessParameterWith(id, symbol_list->children[i], is_signed);
                        break;
                    }
                    case WIRE:
                        if (is_signed) {
                            /* cannot support signed nets right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed nets (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_wire = true;
                        break;
                    case REG:
                        if (is_signed) {
                            /* cannot support signed regs right now */
                            error_message(PARSE_ERROR, symbol_list->children[i]->line_number, current_parse_file,
                                          "Odin does not handle signed regs (%s)\n", symbol_list->children[i]->children[0]->types.identifier);
                        }
                        symbol_list->children[i]->types.variable.is_reg = true;
                        break;
                    case INTEGER:
                        oassert(is_signed && "Integers must always be signed");
                        symbol_list->children[i]->types.variable.is_signed = is_signed;
                        symbol_list->children[i]->types.variable.is_integer = true;
                        break;
                    case GENVAR:
                        oassert(is_signed && "Genvars must always be signed");
                        symbol_list->children[i]->types.variable.is_signed = is_signed;
                        symbol_list->children[i]->types.variable.is_genvar = true;
                        break;
                    default:
                        oassert(false);
                }
            }
        }

        /* parameters don't live in the AST */
        if (top_type != BLOCK && (id == PARAMETER || id == LOCALPARAM)) {
            for (i = 0; i < symbol_list->num_children; i++) {
                symbol_list->children[i]->children[5] = NULL;
            }

            free_whole_tree(symbol_list);
            symbol_list = NULL;
        }
    }

    return symbol_list;
}
/*---------------------------------------------------------------------------------------------
 * (function: newArrayRef)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newArrayRef(char* id, ast_node_t* expression, int line_number) {
    /* allocate or check if there's a node for this */
    ast_node_t* symbol_node = newSymbolNode(id, line_number);
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(ARRAY_REF, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newRangeRef)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newRangeRef(char* id, ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* allocate or check if there's a node for this */
    ast_node_t* symbol_node = newSymbolNode(id, line_number);
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(RANGE_REF, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newPartSelectRangeRef)
 * 
 * NB!! only support [msb:lsb], will always resolve to this syntax
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newMinusColonRangeRef(char* id, ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    ast_node_t* msb = NULL;
    ast_node_t* lsb = NULL;

    if (expression1 == NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "first expression for range ref is NULL %s", id);
    } else if (expression2 == NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "first expression for range ref is NULL  %s", id);
    }

    // expression 1 is the msb here since we subtract expression 2 from it
    msb = expression1;

    ast_node_t* number_one = create_tree_node_number(1L, line_number, current_parse_file);
    ast_node_t* size_to_index = newBinaryOperation(MINUS, expression2, number_one, line_number);

    lsb = newBinaryOperation(MINUS, ast_node_deep_copy(expression1), size_to_index, line_number);

    return newRangeRef(id, msb, lsb, line_number);
}

/*---------------------------------------------------------------------------------------------
 * (function: newPartSelectRangeRef)
 * 
 * NB!! only support [msb:lsb], will always resolve to this syntax
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newPlusColonRangeRef(char* id, ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    ast_node_t* msb = NULL;
    ast_node_t* lsb = NULL;

    if (expression1 == NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "first expression for range ref is NULL %s", id);
    } else if (expression2 == NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "first expression for range ref is NULL  %s", id);
    }

    // expression 1 is the lsb here since we add expression 2 to it
    lsb = expression1;

    ast_node_t* number_one = create_tree_node_number(1L, line_number, current_parse_file);
    ast_node_t* size_to_index = newBinaryOperation(MINUS, expression2, number_one, line_number);

    msb = newBinaryOperation(ADD, ast_node_deep_copy(expression1), size_to_index, line_number);

    return newRangeRef(id, msb, lsb, line_number);
}

/*---------------------------------------------------------------------------------------------
 * (function: newBinaryOperation)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newBinaryOperation(operation_list op_id, ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(BINARY_OPERATION, line_number, current_parse_file);
    /* store the operation type */
    new_node->types.operation.op = op_id;
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1, expression2});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newUnaryOperation)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newUnaryOperation(operation_list op_id, ast_node_t* expression, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(UNARY_OPERATION, line_number, current_parse_file);
    /* store the operation type */
    new_node->types.operation.op = op_id;
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newNegedgeSymbol)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newNegedge(ast_node_t* expression, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(NEGEDGE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newPosedgeSymbol)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newPosedge(ast_node_t* expression, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(POSEDGE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newCaseItem)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newCaseItem(ast_node_t* expression, ast_node_t* statement, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(CASE_ITEM, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression, statement});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newDefaultCase)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newDefaultCase(ast_node_t* statement, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(CASE_DEFAULT, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {statement});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newNonBlocking)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newParallelConnection(ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(SPECIFY_PAL_CONNECTION_STATEMENT, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1, expression2});

    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newNonBlocking)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newNonBlocking(ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(NON_BLOCKING_STATEMENT, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1, expression2});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newInitial)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newInitial(ast_node_t* expression1, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(INITIAL, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1});

    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newBlocking)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newBlocking(ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(BLOCKING_STATEMENT, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1, expression2});

    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newBlocking)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newFunctionAssigning(ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    char* label = vtr::strdup(expression1->types.identifier);

    ast_node_t* node = newSymbolNode(label, line_number);

    expression2->children[1]->children[1]->children[0] = newModuleConnection(NULL, node, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(BLOCKING_STATEMENT, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {expression1, expression2});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newFor)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newFor(ast_node_t* initial, ast_node_t* compare_expression, ast_node_t* terminal, ast_node_t* statement, int line_number) {
    /* create a node for this for reference */
    ast_node_t* new_node = create_node_w_type(FOR, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {initial, compare_expression, terminal, statement});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newWhile)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newWhile(ast_node_t* compare_expression, ast_node_t* statement, int line_number) {
    /* create a node for this for reference */
    ast_node_t* new_node = create_node_w_type(WHILE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {compare_expression, statement});

    /* This needs to be removed once support is added */
    error_message(PARSE_ERROR, line_number, current_parse_file, "%s", "While statements are NOT supported");
    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newIf)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newIf(ast_node_t* compare_expression, ast_node_t* true_expression, ast_node_t* false_expression, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(IF, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {compare_expression, true_expression, false_expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newIfQuestion) for f = a ? b : c;
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newIfQuestion(ast_node_t* compare_expression, ast_node_t* true_expression, ast_node_t* false_expression, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(IF_Q, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {compare_expression, true_expression, false_expression});

    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newCase)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newCase(ast_node_t* compare_expression, ast_node_t* case_list, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(CASE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {compare_expression, case_list});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newAlways)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newAlways(ast_node_t* delay_control, ast_node_t* statement, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(ALWAYS, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {delay_control, statement});

    return new_node;
}

ast_node_t* newStatement(ast_node_t* statement, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(STATEMENT, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {statement});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newModuleConnection)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newModuleConnection(char* id, ast_node_t* expression, int line_number) {
    ast_node_t* symbol_node = NULL;
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(MODULE_CONNECT, line_number, current_parse_file);
    if (id) {
        if (!is_valid_identifier(id)) {
            error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", id);
        }
        symbol_node = newSymbolNode(id, line_number);
    }

    if (expression == NULL) {
        char* number = vtr::strdup("'bz");
        expression = newNumberNode(number, line_number);
    }

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newModuleParameter)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newModuleParameter(char* id, ast_node_t* expression, int line_number) {
    ast_node_t* symbol_node = NULL;
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(MODULE_PARAMETER, line_number, current_parse_file);
    if (id != NULL) {
        if (!is_valid_identifier(id)) {
            error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", id);
        }
        symbol_node = newSymbolNode(id, line_number);
    }

    /* allocate child nodes to this node */
    // leave 4 blank nodes so that expression is the 6th node to behave just like
    // a default var_declare parameter (see create_symbol_table_for_module)
    allocate_children_to_node(new_node, {symbol_node, NULL, NULL, NULL, NULL, expression});

    // set is_parameter for the same reason
    new_node->types.variable.is_parameter = true;

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newModuleNamedInstance)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newModuleNamedInstance(char* unique_name, ast_node_t* module_connect_list, ast_node_t* module_parameter_list, int line_number) {
    if (!is_valid_identifier(unique_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", unique_name);
    }

    ast_node_t* symbol_node = newSymbolNode(unique_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(MODULE_NAMED_INSTANCE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, module_connect_list, module_parameter_list});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newFunctionNamedInstance)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newFunctionNamedInstance(ast_node_t* module_connect_list, ast_node_t* module_parameter_list, int line_number) {
    std::string buffer("function_instance_");
    buffer += std::to_string(size_function_instantiations_by_module);

    char* unique_name = vtr::strdup(buffer.c_str());

    ast_node_t* symbol_node = newSymbolNode(unique_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(FUNCTION_NAMED_INSTANCE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, module_connect_list, module_parameter_list});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newTaskNamedInstance)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newTaskNamedInstance(ast_node_t* module_connect_list, int line_number) {
    std::string buffer("task_instance_");
    buffer += std::to_string(size_task_instantiations_by_module);

    char* unique_name = vtr::strdup(buffer.c_str());

    ast_node_t* symbol_node = newSymbolNode(unique_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(TASK_NAMED_INSTANCE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, module_connect_list});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newTaskInstance)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newTaskInstance(char* task_name, ast_node_t* task_named_instace, ast_node_t* task_parameter_list, int line_number) {
    ast_node_t* symbol_node = newSymbolNode(task_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(TASK_INSTANCE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    add_child_to_node_at_index(task_named_instace, {task_parameter_list}, 2);
    allocate_children_to_node(new_node, {symbol_node, task_named_instace});

    /* store the module symbol name that this calls in a list that will at the end be asociated with the module node */
    task_instantiations_instance_by_module = (ast_node_t**)vtr::realloc(task_instantiations_instance_by_module, sizeof(ast_node_t*) * (size_task_instantiations_by_module + 1));
    task_instantiations_instance_by_module[size_task_instantiations_by_module] = new_node;
    size_task_instantiations_by_module++;

    return new_node;
}

/*-------------------------------------------------------------------------
 * (function: newHardBlockInstance)
 *-----------------------------------------------------------------------*/
ast_node_t* newHardBlockInstance(char* module_ref_name, ast_node_t* module_named_instance, int line_number) {
    if (!is_valid_identifier(module_ref_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", module_ref_name);
    }

    ast_node_t* symbol_node = newSymbolNode(module_ref_name, line_number);

    // create a node for this array reference
    ast_node_t* new_node = create_node_w_type(HARD_BLOCK, line_number, current_parse_file);
    // allocate child nodes to this node
    allocate_children_to_node(new_node, {symbol_node, module_named_instance});

    // mark as hard block
    module_named_instance->type = HARD_BLOCK_NAMED_INSTANCE;

    ast_node_t* connect_list_node = module_named_instance->children[1];
    connect_list_node->type = HARD_BLOCK_CONNECT_LIST;

    for (int i = 0; i < connect_list_node->num_children; i++) {
        connect_list_node->children[i]->type = HARD_BLOCK_CONNECT;
    }

    return new_node;
}

/*-------------------------------------------------------------------------
 * (function: newModuleInstance)
 *-----------------------------------------------------------------------*/
ast_node_t* newModuleInstance(char* module_ref_name, ast_node_t* module_named_instance, int line_number) {
    if (!is_valid_identifier(module_ref_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", module_ref_name);
    }

    long i;
    /* create a node for this array reference */
    ast_node_t* new_master_node = create_node_w_type(MODULE_INSTANCE, line_number, current_parse_file);
    for (i = 0; i < module_named_instance->num_children; i++) {
        /* check if this name was already used */
        long sc_spot = sc_add_string(module_instances_sc, module_named_instance->children[i]->children[0]->types.identifier);
        if (module_instances_sc->data[sc_spot] != NULL) {
            error_message(PARSE_ERROR, line_number, current_parse_file,
                          "Module already has an instance with this name (%s)\n",
                          module_named_instance->children[i]->children[0]->types.identifier);
        }
        module_instances_sc->data[sc_spot] = module_named_instance->children[i];

        ast_node_t* symbol_node = newSymbolNode(module_ref_name, line_number);

        /* create a node for this array reference */
        ast_node_t* new_node = create_node_w_type(MODULE_INSTANCE, line_number, current_parse_file);
        /* allocate child nodes to this node */
        allocate_children_to_node(new_node, {symbol_node, module_named_instance->children[i]});
        if (i == 0)
            allocate_children_to_node(new_master_node, {new_node});
        else
            add_child_to_node(new_master_node, new_node);

        /* store the module symbol name that this calls in a list that will at the end be asociated with the module node */
        sc_add_string(instantiated_modules, module_ref_name);

        /* if this module has already been parsed, update */
        for (int j = 0; j < verilog_ast->top_modules_count; j++) {
            if (sc_lookup_string(instantiated_modules, verilog_ast->top_modules[j]->children[0]->types.identifier) != -1) {
                verilog_ast->top_modules[j]->types.module.is_instantiated = true;
            }
        }
    }

    vtr::free(module_named_instance->children);
    vtr::free(module_named_instance);
    return new_master_node;
}

/*-------------------------------------------------------------------------
 * (function: newFunctionInstance)
 *-----------------------------------------------------------------------*/
ast_node_t* newFunctionInstance(char* function_ref_name, ast_node_t* function_named_instance, int line_number) {
    ast_node_t* symbol_node = newSymbolNode(function_ref_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(FUNCTION_INSTANCE, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, function_named_instance});

    /* store the module symbol name that this calls in a list that will at the end be asociated with the module node */
    function_instantiations_instance_by_module = (ast_node_t**)vtr::realloc(function_instantiations_instance_by_module, sizeof(ast_node_t*) * (size_function_instantiations_by_module + 1));
    function_instantiations_instance_by_module[size_function_instantiations_by_module] = new_node;
    size_function_instantiations_by_module++;

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newGateInstance)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newGateInstance(char* gate_instance_name, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, int line_number) {
    if (!is_valid_identifier(gate_instance_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", gate_instance_name);
    }

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(GATE_INSTANCE, line_number, current_parse_file);
    ast_node_t* symbol_node = NULL;

    if (gate_instance_name != NULL) {
        symbol_node = newSymbolNode(gate_instance_name, line_number);
    }

    char* newChar = vtr::strdup(get_identifier(expression1));
    ast_node_t* newVar = newVarDeclare(newChar, NULL, NULL, NULL, NULL, NULL, line_number);
    ast_node_t* newVarList = newList(VAR_DECLARE_LIST, newVar, line_number);
    ast_node_t* newVarMaked = markAndProcessSymbolListWith(MODULE, WIRE, newVarList, false);
    if (size_module_variables_not_defined == 0) {
        module_variables_not_defined = (ast_node_t**)vtr::calloc(1, sizeof(ast_node_t*));
    } else {
        module_variables_not_defined = (ast_node_t**)vtr::realloc(module_variables_not_defined, sizeof(ast_node_t*) * (size_module_variables_not_defined + 1));
    }
    module_variables_not_defined[size_module_variables_not_defined] = newVarMaked;
    size_module_variables_not_defined++;
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2, expression3});

    return new_node;
}

ast_node_t* newMultipleInputsGateInstance(char* gate_instance_name, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, int line_number) {
    if (!is_valid_identifier(gate_instance_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", gate_instance_name);
    }

    long i;
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(GATE_INSTANCE, line_number, current_parse_file);

    ast_node_t* symbol_node = NULL;

    if (gate_instance_name != NULL) {
        symbol_node = newSymbolNode(gate_instance_name, line_number);
    }

    char* newChar = vtr::strdup(get_identifier(expression1));

    ast_node_t* newVar = newVarDeclare(newChar, NULL, NULL, NULL, NULL, NULL, line_number);

    ast_node_t* newVarList = newList(VAR_DECLARE_LIST, newVar, line_number);

    ast_node_t* newVarMarked = markAndProcessSymbolListWith(MODULE, WIRE, newVarList, false);

    if (size_module_variables_not_defined == 0) {
        module_variables_not_defined = (ast_node_t**)vtr::calloc(1, sizeof(ast_node_t*));
    } else {
        module_variables_not_defined = (ast_node_t**)vtr::realloc(module_variables_not_defined, sizeof(ast_node_t*) * (size_module_variables_not_defined + 1));
    }

    module_variables_not_defined[size_module_variables_not_defined] = newVarMarked;

    size_module_variables_not_defined++;

    allocate_children_to_node(new_node, {symbol_node, expression1, expression2});

    /* allocate n children nodes to this node */
    for (i = 1; i < expression3->num_children; i++)
        add_child_to_node(new_node, expression3->children[i]);

    /* clean up */
    if (expression3->type == MODULE_CONNECT) expression3 = free_single_node(expression3);

    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newGate)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newGate(operation_list op_id, ast_node_t* gate_instance, int line_number) {
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(GATE, line_number, current_parse_file);
    /* store the operation type */
    new_node->types.operation.op = op_id;
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {gate_instance});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newDefparamVarDeclare)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newDefparamVarDeclare(char* symbol, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, ast_node_t* expression4, ast_node_t* value, int line_number) {
    ast_node_t* symbol_node = newSymbolNode(symbol, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(VAR_DECLARE, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2, expression3, expression4, value});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newVarDeclare)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newVarDeclare(char* symbol, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, ast_node_t* expression4, ast_node_t* value, int line_number) {
    if (!is_valid_identifier(symbol)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", symbol);
    }

    ast_node_t* symbol_node = newSymbolNode(symbol, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(VAR_DECLARE, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2, expression3, expression4, value});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: newIntegerTypeVarDeclare)
 *-------------------------------------------------------------------------------------------*/

ast_node_t* newIntegerTypeVarDeclare(char* symbol, ast_node_t* /*expression1*/, ast_node_t* /*expression2*/, ast_node_t* expression3, ast_node_t* expression4, ast_node_t* value, int line_number) {
    if (!is_valid_identifier(symbol)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", symbol);
    }

    ast_node_t* symbol_node = newSymbolNode(symbol, line_number);

    ast_node_t* number_node_with_value_31 = create_tree_node_number(31L, line_number, current_parse_file);

    ast_node_t* number_node_with_value_0 = create_tree_node_number(0L, line_number, current_parse_file);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(VAR_DECLARE, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, number_node_with_value_31, number_node_with_value_0, expression3, expression4, value});

    return new_node;
}
/*-----------------------------------------
 * ----------------------------------------------------
 * (function: newModule)
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newModule(char* module_name, ast_node_t* list_of_parameters, ast_node_t* list_of_ports, ast_node_t* list_of_module_items, int line_number) {
    ast_node_t* new_node = NULL;
    long sc_spot = sc_lookup_string(hard_block_names, module_name);

    if (!is_valid_identifier(module_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "Invalid character in identifier (%s)\n",
                      module_name);
    } else if (sc_spot != -1
               || !strcmp(module_name, SINGLE_PORT_RAM_string)
               || !strcmp(module_name, DUAL_PORT_RAM_string)) {
        error_message(PARSE_ERROR, line_number, current_parse_file,
                      "Module name collides with hard block of the same name (%s)\n", module_name);
    } else if (list_of_ports == NULL
               || list_of_ports->num_children == 0) {
        // this may change with hierarchy but for now we simply delete it
        warning_message(PARSE_ERROR, line_number, current_parse_file,
                        "there are no ports for the module (%s)\n\tall logic will be dropped since it is not driving an output\n",
                        module_name);
        vtr::free(module_name);
        free_whole_tree(list_of_parameters);
        free_whole_tree(list_of_ports);
        free_whole_tree(list_of_module_items);
    } else {
        /* create a node for this array reference */
        new_node = create_node_w_type(MODULE, line_number, current_parse_file);
        ast_node_t* symbol_node = newSymbolNode(module_name, line_number);
        /* mark all the ports symbols as ports */
        ast_node_t* port_declarations = resolve_ports(MODULE, list_of_ports);

        /* ports are expected to be in module items */
        if (port_declarations) {
            add_child_to_node_at_index(list_of_module_items, port_declarations, 0);
        }

        /* parameters don't live in the AST */
        if (list_of_parameters) {
            for (long i = 0; i < list_of_parameters->num_children; i++) {
                list_of_parameters->children[i]->children[5] = NULL;
            }
            free_whole_tree(list_of_parameters);
        }

        /* allocate child nodes to this node */
        allocate_children_to_node(new_node, {symbol_node, list_of_ports, list_of_module_items});

        /* check if this module has been instantiated */
        if (new_node->types.module.is_instantiated == false) {
            new_node->types.module.is_instantiated = (sc_lookup_string(instantiated_modules, module_name) != -1);
        }
        new_node->types.scope = pop_scope();

        new_node->types.function.function_instantiations_instance = function_instantiations_instance_by_module;
        new_node->types.function.size_function_instantiations = size_function_instantiations_by_module;
        new_node->types.function.is_instantiated = false;

        new_node->types.task.task_instantiations_instance = task_instantiations_instance_by_module;
        new_node->types.task.size_task_instantiations = size_task_instantiations_by_module;
        new_node->types.task.is_instantiated = false;

        /* record this module in the list of modules (for evaluation later in terms of just nodes) */
        add_top_module_to_ast(verilog_ast, new_node);
        for (long i = 0; i < size_module_variables_not_defined; i++) {
            short variable_found = false;
            for (long j = 0; j < list_of_module_items->num_children && variable_found == false; j++) {
                if (list_of_module_items->children[j]->type == VAR_DECLARE_LIST) {
                    for (long k = 0; k < list_of_module_items->children[j]->num_children; k++) {
                        if (strcmp(list_of_module_items->children[j]->children[k]->children[0]->types.identifier, module_variables_not_defined[i]->children[0]->children[0]->types.identifier) == 0)
                            variable_found = true;
                    }
                }
            }
            if (!variable_found)
                add_child_to_node_at_index(list_of_module_items, module_variables_not_defined[i], 0);
            else
                free_whole_tree(module_variables_not_defined[i]);
        }

        /* clean up */
        vtr::free(module_variables_not_defined);
        sc_spot = sc_add_string(module_names_to_idx, module_name);
        if (module_names_to_idx->data[sc_spot] != NULL) {
            error_message(PARSE_ERROR, line_number, current_parse_file, "module names with the same name -> %s\n", module_name);
        }
        /* store the data which is an idx here */
        module_names_to_idx->data[sc_spot] = (void*)new_node;

        /* now that we've bottom up built the parse tree for this module, go to the next module */
        next_module();
    }
    return new_node;
}

/*-----------------------------------------
 * ----------------------------------------------------
 * (function: newFunction)
 *-------------------------------------------------------------------------------------------*/

ast_node_t* newFunction(ast_node_t* function_return, ast_node_t* list_of_ports, ast_node_t* list_of_module_items, int line_number, bool automatic) {
    long i, j;
    long sc_spot;
    char* label = NULL;
    ast_node_t* var_node = NULL;
    ast_node_t* symbol_node = NULL;

    if (automatic) {
        warning_message(PARSE_ERROR, line_number, current_parse_file, "%s", "ODIN II does not (yet) differentiate between automatic and static tasks & functions.IGNORING ");
    }

    if (function_return->children[0]->types.variable.is_integer) {
        warning_message(PARSE_ERROR, line_number, current_parse_file, "%s", "ODIN_II does not fully support input/output integers in functions");
    }

    if (list_of_ports == NULL) {
        list_of_ports = create_node_w_type(VAR_DECLARE_LIST, line_number, current_parse_file);
    }

    label = vtr::strdup(function_return->children[0]->children[0]->types.identifier);

    var_node = newVarDeclare(label, NULL, NULL, NULL, NULL, NULL, line_number);

    add_child_to_node_at_index(list_of_ports, var_node, 0);

    char* function_name = vtr::strdup(function_return->children[0]->children[0]->types.identifier);

    if (!is_valid_identifier(function_name)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", function_name);
    }

    add_child_to_node_at_index(list_of_module_items, function_return, 0);

    for (i = 0; i < list_of_module_items->num_children; i++) {
        if (list_of_module_items->children[i]->type == VAR_DECLARE_LIST) {
            for (j = 0; j < list_of_module_items->children[i]->num_children; j++) {
                if (list_of_module_items->children[i]->children[j]->types.variable.is_input) {
                    label = vtr::strdup(list_of_module_items->children[i]->children[j]->children[0]->types.identifier);
                    var_node = newVarDeclare(label, NULL, NULL, NULL, NULL, NULL, line_number);
                    newList_entry(list_of_ports, var_node);
                }
            }
        }
    }

    symbol_node = newSymbolNode(function_name, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(FUNCTION, line_number, current_parse_file);

    /* mark all the ports symbols as ports */
    ast_node_t* port_declarations = resolve_ports(FUNCTION, list_of_ports);
    if (port_declarations) {
        add_child_to_node_at_index(list_of_module_items, port_declarations, 0);
    }

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, list_of_ports, list_of_module_items});

    /* store the list of functions this function instantiates */
    new_node->types.scope = pop_scope();
    new_node->types.function.function_instantiations_instance = function_instantiations_instance;
    new_node->types.function.size_function_instantiations = size_function_instantiations;
    new_node->types.function.is_instantiated = false;

    /* record this module in the list of modules (for evaluation later in terms of just nodes) */
    ast_functions = (ast_node_t**)vtr::realloc(ast_functions, sizeof(ast_node_t*) * (num_functions + 1));
    ast_functions[num_functions] = new_node;
    sc_spot = sc_add_string(module_names_to_idx, function_name);
    if (module_names_to_idx->data[sc_spot] != NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "module names with the same name -> %s\n", function_name);
    }
    /* store the data which is an idx here */
    module_names_to_idx->data[sc_spot] = (void*)new_node;

    /* now that we've bottom up built the parse tree for this module, go to the next module */
    next_function();

    return new_node;
}

ast_node_t* newTask(char* task_name, ast_node_t* list_of_ports, ast_node_t* list_of_task_items, int line_number, bool automatic) {
    long sc_spot;
    ast_node_t* symbol_node = newSymbolNode(task_name, line_number);
    ast_node_t* var_node = NULL;
    char* label = NULL;

    if (automatic) {
        warning_message(PARSE_ERROR, line_number, 0, "%s", "ODIN II does not (yet) differentiate between automatic and static tasks & functions. IGNORING");
    }

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(TASK, line_number, current_parse_file);

    for (int i = 0; i < list_of_task_items->num_children; i++) {
        if (list_of_task_items->children[i]->type == VAR_DECLARE_LIST) {
            for (int j = 0; j < list_of_task_items->children[i]->num_children; j++) {
                if (list_of_task_items->children[i]->children[j]->types.variable.is_input) {
                    label = vtr::strdup(list_of_task_items->children[i]->children[j]->children[0]->types.identifier);
                    var_node = newVarDeclare(label, NULL, NULL, NULL, NULL, NULL, line_number);
                    var_node->types.variable.is_input = true;
                    if (list_of_ports) {
                        newList_entry(list_of_ports, var_node);
                    } else {
                        list_of_ports = newList(VAR_DECLARE_LIST, var_node, line_number);
                    }
                } else if (list_of_task_items->children[i]->children[j]->types.variable.is_output) {
                    label = vtr::strdup(list_of_task_items->children[i]->children[j]->children[0]->types.identifier);
                    var_node = newVarDeclare(label, NULL, NULL, NULL, NULL, NULL, line_number);
                    var_node->types.variable.is_output = true;
                    if (list_of_ports) {
                        newList_entry(list_of_ports, var_node);
                    } else {
                        list_of_ports = newList(VAR_DECLARE_LIST, var_node, line_number);
                    }
                } else if (list_of_task_items->children[i]->children[j]->types.variable.is_inout) {
                    label = vtr::strdup(list_of_task_items->children[i]->children[j]->children[0]->types.identifier);
                    var_node = newVarDeclare(label, NULL, NULL, NULL, NULL, NULL, line_number);
                    var_node->types.variable.is_inout = true;
                    if (list_of_ports) {
                        newList_entry(list_of_ports, var_node);
                    } else {
                        list_of_ports = newList(VAR_DECLARE_LIST, var_node, line_number);
                    }
                }
            }
        }
    }

    ast_node_t* port_declarations = resolve_ports(TASK, list_of_ports);
    /* ports are expected to be in module items */
    if (port_declarations) {
        add_child_to_node_at_index(list_of_task_items, port_declarations, 0);
    }

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, list_of_ports, list_of_task_items});

    new_node->types.scope = pop_scope();
    /* store the list of tasks and functions this task instantiates */
    new_node->types.task.task_instantiations_instance = task_instantiations_instance;
    new_node->types.task.size_task_instantiations = size_task_instantiations;
    new_node->types.task.is_instantiated = false;

    new_node->types.function.function_instantiations_instance = function_instantiations_instance;
    new_node->types.function.size_function_instantiations = size_function_instantiations;
    new_node->types.function.is_instantiated = false;

    /* record this module in the list of modules (for evaluation later in terms of just nodes) */
    ast_tasks = (ast_node_t**)vtr::realloc(ast_tasks, sizeof(ast_node_t*) * (num_tasks + 1));
    ast_tasks[num_tasks] = new_node;

    sc_spot = sc_add_string(module_names_to_idx, task_name);
    if (module_names_to_idx->data[sc_spot] != NULL) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "task names with the same name -> %s\n", task_name);
    }

    /* store the data which is an idx here */
    module_names_to_idx->data[sc_spot] = (void*)new_node;

    /* now that we've bottom up built the parse tree for this task, go to the next task */
    next_task();

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: next_task)
 *-------------------------------------------------------------------------------------------*/
void next_task() {
    num_tasks++;

    /* create a new list for the instantiations list */
    task_instantiations_instance = NULL;
    size_task_instantiations = 0;

    /* old ones are done so clean */
    sc_free_string_cache(tasks_inputs_sc);
    sc_free_string_cache(tasks_outputs_sc);
    /* make for next task */
    tasks_inputs_sc = sc_new_string_cache();
    tasks_outputs_sc = sc_new_string_cache();
}
/*---------------------------------------------------------------------------------------------
 * (function: next_function)
 *-------------------------------------------------------------------------------------------*/
void next_function() {
    num_functions++;

    /* create a new list for the instantiations list */
    function_instantiations_instance = NULL;
    size_function_instantiations = 0;

    /* old ones are done so clean */
    sc_free_string_cache(functions_inputs_sc);
    sc_free_string_cache(functions_outputs_sc);
    /* make for next function */
    functions_inputs_sc = sc_new_string_cache();
    functions_outputs_sc = sc_new_string_cache();
}
/*---------------------------------------------------------------------------------------------
 * (function: next_module)
 *-------------------------------------------------------------------------------------------*/
void next_module() {
    num_modules++;
    num_functions = 0;

    /* define the string cache for the next module */
    function_instantiations_instance_by_module = NULL;
    size_function_instantiations_by_module = 0;

    module_variables_not_defined = NULL;
    size_module_variables_not_defined = 0;
    /* old ones are done so clean */
    sc_free_string_cache(modules_inputs_sc);
    sc_free_string_cache(modules_outputs_sc);
    sc_free_string_cache(module_instances_sc);
    /* make for next module */
    modules_inputs_sc = sc_new_string_cache();
    modules_outputs_sc = sc_new_string_cache();
    module_instances_sc = sc_new_string_cache();
}

/*--------------------------------------------------------------------------
 * (function: newDefparam)
 *------------------------------------------------------------------------*/
ast_node_t* newDefparam(ids top_type, ast_node_t* val, int line_number) {
    ast_node_t* new_node = NULL;

    if (val) {
        for (int i = 0; i < val->num_children; i++) {
            new_node = val->children[i];

            new_node->type = MODULE_PARAMETER;
            new_node->types.variable.is_defparam = true;
            new_node->children[5]->types.variable.is_defparam = true;
            new_node->line_number = line_number;

            /* defparams within named or generate blocks are handled during elaboration */
            if (top_type != BLOCK) {
                long sc_spot = sc_add_string(current_scope->defparam_sc, new_node->children[0]->types.identifier);
                current_scope->defparam_sc->data[sc_spot] = (void*)new_node;
            }
        }
    }

    return val;
}

/* --------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------
 * AST VIEWING FUNCTIONS
 * --------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------*/

int unique_label_count;
/*---------------------------------------------------------------------------
 * (function: graphVizOutputAst)
 *-------------------------------------------------------------------------*/
void graphVizOutputAst(std::string path, ast_node_t* top) {
    char path_and_file[4096];
    FILE* fp;
    static int file_num = 0;

    /* open the file */
    odin_sprintf(path_and_file, "%s/%s_ast.dot", path.c_str(), top->children[0]->types.identifier);
    fp = fopen(path_and_file, "w");
    file_num++;

    /* open graph */
    fprintf(fp, "digraph G {\t\nranksep=.25;\n");
    unique_label_count = 0;

    graphVizOutputAst_traverse_node(fp, top, NULL, -1);

    /* close graph */
    fprintf(fp, "}\n");
    fclose(fp);
}

/*---------------------------------------------------------------------------------------------
 * (function: graphVizOutputAst_traverse_node)
 *-------------------------------------------------------------------------------------------*/
void graphVizOutputAst_traverse_node(FILE* fp, ast_node_t* node, ast_node_t* from, int from_num) {
    if (!node)
        return;

    /* increase the unique count for other nodes since ours is recorded */
    int my_label = unique_label_count++;

    fprintf(fp, "\t%d [label=\"", my_label);
    fprintf(fp, "%s", ids_STR[node->type]);
    switch (node->type) {
        case VAR_DECLARE: {
            std::stringstream temp;
            if (node->types.variable.is_input) temp << " INPUT";
            if (node->types.variable.is_output) temp << " OUTPUT";
            if (node->types.variable.is_inout) temp << " INOUT";
            if (node->types.variable.is_port) temp << " PORT";
            if (node->types.variable.is_parameter) temp << " PARAMETER";
            if (node->types.variable.is_wire) temp << " WIRE";
            if (node->types.variable.is_reg) temp << " REG";

            fprintf(fp, ": %s", temp.str().c_str());
            break;
        }
        case NUMBERS:
            fprintf(fp, ": %s (%s)",
                    node->types.vnumber->to_bit_string().c_str(),
                    node->types.vnumber->to_printable().c_str());
            break;

        case UNARY_OPERATION: //fallthrough
        case BINARY_OPERATION:
            fprintf(fp, ": %s", name_based_on_op(node->types.operation.op));
            break;

        case IDENTIFIERS:
            fprintf(fp, ": %s", node->types.identifier);
            break;

        default:
            break;
    }
    fprintf(fp, "\"];\n");

    /* print out the connection with the previous node */
    if (from != NULL)
        fprintf(fp, "\t%d -> %d;\n", from_num, my_label);

    for (long i = 0; i < node->num_children; i++) {
        graphVizOutputAst_traverse_node(fp, node->children[i], node, my_label);
    }
}
/*---------------------------------------------------------------------------------------------
 * (function: newVarDeclare) for 2D Array
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newVarDeclare2D(char* symbol, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, ast_node_t* expression4, ast_node_t* expression5, ast_node_t* expression6, ast_node_t* value, int line_number) {
    if (!is_valid_identifier(symbol)) {
        error_message(PARSE_ERROR, line_number, current_parse_file, "Invalid character in identifier (%s)\n", symbol);
    }
    ast_node_t* symbol_node = newSymbolNode(symbol, line_number);

    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(VAR_DECLARE, line_number, current_parse_file);

    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2, expression3, expression4, expression5, expression6, value});
    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newArrayRef) for 2D Array
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newArrayRef2D(char* id, ast_node_t* expression1, ast_node_t* expression2, int line_number) {
    /* allocate or check if there's a node for this */
    ast_node_t* symbol_node = newSymbolNode(id, line_number);
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(ARRAY_REF, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2});
    return new_node;
}
/*---------------------------------------------------------------------------------------------
 * (function: newRangeRef) for 2D Array
 *-------------------------------------------------------------------------------------------*/
ast_node_t* newRangeRef2D(char* id, ast_node_t* expression1, ast_node_t* expression2, ast_node_t* expression3, ast_node_t* expression4, int line_number) {
    /* allocate or check if there's a node for this */
    ast_node_t* symbol_node = newSymbolNode(id, line_number);
    /* create a node for this array reference */
    ast_node_t* new_node = create_node_w_type(RANGE_REF, line_number, current_parse_file);
    /* allocate child nodes to this node */
    allocate_children_to_node(new_node, {symbol_node, expression1, expression2, expression3, expression4});

    return new_node;
}

/*---------------------------------------------------------------------------------------------
 * (function: is_valid_identifier) for identifiers
 *-------------------------------------------------------------------------------------------*/
bool is_valid_identifier(char* str) {
    if (str) {
        int i = 0;
        char ch = str[i];
        while (ch != 0) {
            if (ch == '.') {
                return false;
            }
            ch = str[++i];
        }
    }
    return true;
}
