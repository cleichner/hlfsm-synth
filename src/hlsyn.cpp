// ECE 474A/574A - Example C++ program with support for checking commandline
//                 arguments required for Assignment 4
// Author: Charles Leichner
// Date: 12/5/12

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <vector>
#include <climits>
#include <cfloat>
using namespace std;

class Symbol;
typedef map<string, Symbol> SymbolTable;

typedef int Time;

bool DEBUG = false;

typedef unsigned int state;
state unique_state() {
    static state current_state = 0;
    return current_state++;
}

state unique_end_state() {
    static state current_end_state = UINT_MAX;
    return current_end_state--;
}

void true_or_die(bool constraint, string error_message) {
    if (!constraint) {
        cerr << "Error: " << error_message << '\n';
        exit(-1);
    }
}

bool is_not_alphanumeric(char c) {
    return !(isalpha(c) || isdigit(c));
}

bool all_alphanumeric(string str) {
    return find_if(str.begin(), str.end(), is_not_alphanumeric) == str.end();
}

string alphanumeric(string str) {
    true_or_die(all_alphanumeric(str),
            "identifiers must consist of letters and numbers");
    return str;
}

long unique() {
    static long n = 0;
    return n++;
}

typedef enum {
    EMPTY,
    VAR,
    DO,
    WHILE,
    IF,
    ASSIGN,
    PLUS,
    MINUS,
    MULT,
    LT,
    GT,
    EQ,
    QMARK,
    COLON,
    L_SHIFT,
    R_SHIFT,
    INPUTS,
    OUTPUTS,
    REGS,
    L_CURLY,
    R_CURLY,
    L_PAREN,
    R_PAREN,
    NEWLINE,
    EOF_TOKEN,
} TokenType;

class Token {
   friend ostream& operator<<(ostream& output, const Token& t);
  public:
    TokenType type;
    string contents;

    Token() : type(EMPTY), contents() {}
    Token(TokenType type, string contents) : type(type), contents(contents) {}

    static Token newline() {
        return Token(NEWLINE, "\n");
    }

    static Token eof() {
        return Token(EOF_TOKEN, "eof");
    }

    static Token from(string s) {
        if (s == ")") {
            return Token(R_PAREN, s);
        } else if (s == "(") {
            return Token(L_PAREN, s);
        } else if (s == "}") {
            return Token(R_CURLY, s);
        } else if (s == "{") {
            return Token(L_CURLY, s);
        } else if (s == "INPUTS") {
            return Token(INPUTS, s);
        } else if (s == "OUTPUTS") {
            return Token(OUTPUTS, s);
        } else if (s == "REGS") {
            return Token(REGS, s);
        } else if (s == ">>") {
            return Token(R_SHIFT, s);
        } else if (s == "<<") {
            return Token(L_SHIFT, s);
        } else if (s == ":") {
            return Token(COLON, s);
        } else if (s == "?") {
            return Token(QMARK, s);
        } else if (s == "==") {
            return Token(EQ, s);
        } else if (s == ">") {
            return Token(GT, s);
        } else if (s == "<") {
            return Token(LT, s);
        } else if (s == "*") {
            return Token(MULT, s);
        } else if (s == "-") {
            return Token(MINUS, s);
        } else if (s == "+") {
            return Token(PLUS, s);
        } else if (s == "=") {
            return Token(ASSIGN, s);
        } else if (s == "if") {
            return Token(IF, s);
        } else if (s == "while") {
            return Token(WHILE, s);
        } else if (s == "do") {
            return Token(DO, s);
        } else {
            true_or_die(all_alphanumeric(s),
                "variables must consist only of alphanumeric characters");
            return Token(VAR, s);
        }
    }

    bool is_op() {
        return this->type == PLUS ||
            this->type == MINUS ||
            this->type == MULT ||
            this->type == LT ||
            this->type == GT ||
            this->type == EQ ||
            this->type == QMARK ||
            this->type == COLON ||
            this->type == L_SHIFT ||
            this->type == R_SHIFT;
    }

};

std::ostream& operator<< (std::ostream& os, const Token& t) {
    switch (t.type) {
        case VAR: os << "VAR"; break;
        case DO: os << "DO"; break;
        case WHILE: os << "WHILE"; break;
        case IF: os << "IF"; break;
        case ASSIGN: os << "ASSIGN"; break;
        case PLUS: os << "PLUS"; break;
        case MINUS: os << "MINUS"; break;
        case MULT: os << "MULT"; break;
        case LT: os << "LT"; break;
        case GT: os << "GT"; break;
        case EQ: os << "EQ"; break;
        case QMARK: os << "QMARK"; break;
        case COLON: os << "COLON"; break;
        case L_SHIFT: os << "L_SHIFT"; break;
        case R_SHIFT: os << "R_SHIFT"; break;
        case INPUTS: os << "INPUTS"; break;
        case OUTPUTS: os << "OUTPUTS"; break;
        case REGS: os << "REGS"; break;
        case L_CURLY: os << "L_CURLY"; break;
        case R_CURLY: os << "R_CURLY"; break;
        case L_PAREN: os << "L_PAREN"; break;
        case R_PAREN: os << "R_PAREN"; break;
        case EMPTY: os << "EMPTY"; break;
        case NEWLINE: os << "NEWLINE"; break;
        case EOF_TOKEN: os << "EOF_TOKEN"; break;
        default:
            true_or_die(false, "unknown token!");
    }
    return os;
}

class TokenStream {
  public:
    TokenStream(istream& in) : in(in), current(Token(EOF_TOKEN, "EOF")),
                               c_line(NULL), word(split.end()) {
        ++(*this);
    }

    Token& operator*() {
        return current;
    }

    Token* operator->() {
        return &current;
    }

    TokenStream& operator ++(int) {
        return ++(*this);
    }

    // wraps Token::from to ignore comments
    Token next_token() {
        if (*word == "//") {
            word = split.end();
            ++(*this);
        }
        if (DEBUG) {
            cout << Token::from(*word) << " -> " << *word << '\n';
        }
        return Token::from(*word);
    }

    TokenStream& operator ++() {
        if (word != split.end()) {
            ++word;
        }
        if (word != split.end()) {
            current = next_token();
        } else if (getline(in, line)) {
            delete c_line;
            c_line = new stringstream(line);
            line_start = istream_iterator<string>(*c_line);
            split = vector<string>(line_start, line_end);
            word = split.begin();
            if (word != split.end()) {
                current = next_token();
            } else {
                if (DEBUG) {
                    cout << "NEWLINE\n";
                }
                current = Token::newline();
            }
        } else {
            if (DEBUG) {
                cout << "EOF\n";
            }
            current = Token::eof();
        }
        return *this;
    }

    ~TokenStream() {
        delete c_line;
    }

  private:

    // Disallow copy and assign
    TokenStream(const TokenStream&);
    void operator=(const TokenStream&);

    istream& in;
    Token current;

    // used for splitting on/removing whitespace
    string line;
    stringstream* c_line;
    istream_iterator<string> line_start;
    istream_iterator<string> line_end;
    vector<string> split;
    vector<string>::iterator word;
};

typedef enum {
    NONE,
    INPUT,
    OUTPUT,
    REG,
} SymbolType;

class Symbol {
    friend std::ostream& operator<< (std::ostream& os, const Symbol& s);
  public:
    string name;
    SymbolType type;

    Symbol(string name, SymbolType t) : name(name), type(t) {}
    Symbol() : name("NONE"), type(NONE) {}

    Symbol(const Symbol& that) : name(that.name), type(that.type) {}

    Symbol& operator=(const Symbol& other) {
        this->name = other.name;
        this->type = other.type;
        return *this;
    }
};

std::ostream& operator<< (std::ostream& os, const Symbol& s) {
    switch (s.type) {
        case INPUT: os << "INPUT"; break;
        case OUTPUT: os << "OUTPUT"; break;
        case REG: os << "REG"; break;
        default:
            true_or_die(false, "unknown symbol type");
    }
    return os;
}

class ExpressionBlock;

// Base policy for scheduling blocks of arithmatic expressions
class Schedule {
  public:
    virtual vector<ExpressionBlock> operator()(ExpressionBlock, SymbolTable) = 0;
    virtual ~Schedule() {};
};

class Statements {
  public:
    vector<Statements*> contents;

    Statements() {}
    Statements(const Statements& that) : contents(that.contents) {}
    virtual ~Statements() {
        for (vector<Statements*>::iterator it = contents.begin();
            it != contents.end(); ++it) {
            delete *it;
        }
    }

    virtual void generate_fsm(SymbolTable symbols, ostream& os, Schedule* schedule,
                              state start,  state end) {
        state inner_start = start;
        state inner_end;

        if (!contents.empty()) {
            for (vector<Statements*>::iterator it = contents.begin();
                it != contents.end(); ++it) {
                if (it+1 == contents.end()) {
                    inner_end = end;
                } else {
                    inner_end = unique_state();
                }
                (*it)->generate_fsm(symbols, os, schedule, inner_start, inner_end);
                inner_start = inner_end;
            }
        } else {
            os << "        " << start << ": begin\n";
            os << "            State <= " << end << ";\n";
            os << "        end\n";
        }
    }

    void append(Statements* s) {
        contents.push_back(s);
    }
};

// individual expression
class Expression {
    friend std::ostream& operator<< (std::ostream& os, const Expression& s);
  public:
    TokenType op;
    Symbol result;
    vector<Symbol> arguments;

    Expression() : op(EMPTY) {}
    Expression(const Expression& that) :
        op(that.op), result(that.result), arguments(that.arguments) {}

    void add_argument(Symbol s) {
        arguments.push_back(s);
    }

    bool operator<(Expression other) const {
        if (result.name < other.result.name) {
            return true;
        } else if (result.name == other.result.name) {
            // if the result is the same, compare the arguments
            vector<Symbol>::const_iterator o_it = other.arguments.begin();
            vector<Symbol>::const_iterator o_end = other.arguments.end();

            for (vector<Symbol>::const_iterator it = arguments.begin();
                 it != arguments.end(); ++it) {
                // other expression is shorter
                if (o_it == o_end) {
                    return false;
                }

                if (it->name < o_it->name) {
                    return true;
                }

                if (o_it != o_end) {
                    ++o_it;
                }
            }

            // this expression has fewer arguments
            if (o_it != o_end) {
                return true;
            }

            // compare on operator
            if (op < other.op) {
                return true;
            }

            return false;
        } else {
            return false;
        }
    };
};

std::ostream& operator<< (std::ostream& os, const Expression& s) {
    os << s.result.name << " ";
    switch (s.op) {
        case ASSIGN: os << "="; break;
        case PLUS: os << "+"; break;
        case MINUS: os << "-"; break;
        case MULT: os << "*"; break;
        case LT: os << "<"; break;
        case GT: os << ">"; break;
        case EQ: os << "=="; break;
        case QMARK: os << "?"; break;
        case L_SHIFT: os << "<<"; break;
        case R_SHIFT: os << ">>"; break;
        default:
            true_or_die(false, "unknown operator");
    }

    for (vector<Symbol>::const_iterator arg = s.arguments.begin();
         arg != s.arguments.end(); ++arg) {
        os << " " << arg->name;
    }

    return os;
}

// Schedulable block of contiguous expressions
class ExpressionBlock : public Statements {
  public:
    vector<Expression> expressions;
    ExpressionBlock() : Statements() {}

    ExpressionBlock(const ExpressionBlock& that) :
        Statements(), expressions(that.expressions) {}

    virtual Statements* statements() {
        return this;
    }
    void add_expression(Expression e) {
        expressions.push_back(e);
    }
    vector<Expression>::iterator begin() {
        return expressions.begin();
    }
    vector<Expression>::iterator end() {
        return expressions.end();
    }
    vector<Expression>::reverse_iterator rbegin() {
        return expressions.rbegin();
    }
    vector<Expression>::reverse_iterator rend() {
        return expressions.rend();
    }
    virtual void generate_fsm(SymbolTable symbols, ostream& os, Schedule* schedule,
                              state start,  state end) {
        state inner_start = start;
        state inner_end;

        os << "\n        // STATEMENTS\n";
        vector<ExpressionBlock> scheduled_blocks = (*schedule)(*this, symbols);
        for (vector<ExpressionBlock>::iterator block = scheduled_blocks.begin();
            block != scheduled_blocks.end(); ++block) {
            os << "        " << inner_start << ": begin\n";
            for (vector<Expression>::iterator contents = block->begin();
                 contents != block->end(); ++contents) {
                os << "            " << contents->result.name << " <= ";
                vector<Symbol>::iterator arguments = contents->arguments.begin();
                os << arguments->name;
                if (arguments != contents->arguments.end()) {
                    ++arguments;
                    if (arguments != contents->arguments.end()) {
                        os << " ";
                        switch (contents->op) {
                            case PLUS: os << "+"; break;
                            case MINUS: os << "-"; break;
                            case MULT: os << "*"; break;
                            case LT: os << "<"; break;
                            case GT: os << ">"; break;
                            case EQ: os << "=="; break;
                            case QMARK: os << "?"; break;
                            case L_SHIFT: os << "<<"; break;
                            case R_SHIFT: os << ">>"; break;
                            default:
                                true_or_die(false,
                                     "unknown operator in generate_fsm");
                        }
                        os << " " << arguments->name;
                        ++arguments;
                        if (arguments != contents->arguments.end()) {
                            os << " : " <<  arguments->name;
                        }
                    }
                }
                os << ";\n";
            }
            if (block+1 == scheduled_blocks.end()) {
                inner_end = end;
            } else {
                inner_end = unique_state();
            }
            os << "            State <= "<< inner_end <<";\n";
            os << "        end\n";
            inner_start = inner_end;
        }
        os << '\n';
    }
};

class While : public Statements {
  public:
    Symbol condition; // contents of While is Statements::statements

    virtual void generate_fsm(SymbolTable symbols, ostream& os, Schedule* schedule, state start, state end) {
        state inner_start = unique_state();
        state inner_end = unique_end_state();

        os << "\n        // WHILE STATEMENT\n";
        os << "        " << start << ": begin\n";
        os << "            if (" << condition.name << ") begin\n";
        os << "                State <= " << inner_start << ";\n";
        os << "            end else begin\n";
        os << "                State <= " << end << ";\n";
        os << "            end\n";
        os << "        end\n";

        Statements::generate_fsm(symbols, os, schedule, inner_start, inner_end);

        os << "        " << inner_end << ": begin\n";
        os << "            State <= " << start << ";\n";
        os << "        end // END WHILE STATEMENT\n\n";
    }
};

class Do : public Statements {
  public:
    Symbol condition; // contents of Do is Statements::statements

    virtual void generate_fsm(SymbolTable symbols, ostream& os, Schedule* schedule, state start, state end) {
        state inner_start = unique_state();
        state inner_end = unique_end_state();

        os << "\n        // DO-WHILE STATEMENT\n";
        os << "        " << start << ": begin\n";
        os << "            State <= " << inner_start << ";\n";
        os << "        end\n";

        Statements::generate_fsm(symbols, os, schedule, inner_start, inner_end);

        os << "        " << inner_end << ": begin\n";
        os << "            if (" << condition.name << ") begin\n";
        os << "                State <= " << start << ";\n";
        os << "            end else begin\n";
        os << "                State <= " << end << ";\n";
        os << "            end\n";
        os << "        end // END DO-WHILE STATEMENT\n\n";
    }
};

class If : public Statements {
  public:
    Symbol condition; // contents of If is Statements::statements

    virtual void generate_fsm(SymbolTable symbols, ostream& os, Schedule* schedule, state start, state end) {
        state inner_start = unique_state();
        state inner_end = unique_end_state();

        os << "\n        // IF STATEMENT\n";
        os << "        " << start << ": begin\n";
        os << "            if (" << condition.name << ") begin\n";
        os << "                State <= " << inner_start << ";\n";
        os << "            end else begin\n";
        os << "                State <= " << inner_end << ";\n";
        os << "            end\n";
        os << "        end\n";

        Statements::generate_fsm(symbols, os, schedule, inner_start, inner_end);

        os << "        " << inner_end << ": begin\n";
        os << "            State <= " << end << ";\n";
        os << "        end // END IF STATEMENT\n\n";
    }
};

class NoSchedule : public Schedule {
  public:
    virtual vector<ExpressionBlock> operator()(ExpressionBlock e, SymbolTable) {
        vector<ExpressionBlock> scheduled_blocks;
        for (vector<Expression>::iterator it = e.begin();
            it != e.end(); ++it) {
            ExpressionBlock block;
            block.add_expression(*it);
            scheduled_blocks.push_back(block);
        }
        return scheduled_blocks;
    }
};


typedef map<Expression, Time> Sequence;

map<Expression, set<Expression> > predecessors_from_expression_block(ExpressionBlock e) {
    map<Expression, set<Expression> > pred;
    for (vector<Expression>::reverse_iterator expression = e.rbegin();
        expression != e.rend(); ++expression) {
        pred[*expression] = set<Expression>();
        for (vector<Symbol>::iterator arg = expression->arguments.begin();
            arg != expression->arguments.end(); ++arg) {
            true_or_die(expression->result.name != arg->name,
                        "self-reference in expression!");
            for (vector<Expression>::reverse_iterator dep = expression;
                dep != e.rend(); ++dep) {
                if (arg->name == dep->result.name) {
                    pred[*expression].insert(*dep);
                }
            }
        }
    }
    return pred;
}

class ASAPSchedule : public Schedule {
  public:

    vector<ExpressionBlock> sequence_to_expression_blocks(
        Sequence sequence) {
        bool still_adding = true;
        vector<ExpressionBlock> scheduled_expressions;
        Time t = 1;
        while (still_adding) {
            still_adding = false;
            ExpressionBlock e;
            for (Sequence::iterator it = sequence.begin();
                it != sequence.end(); ++it) {
                if (it->second == t) {
                    e.add_expression(it->first);
                    still_adding = true;
                }
            }
            if (still_adding) {
                scheduled_expressions.push_back(e);
            }
            t++;
        }
        return scheduled_expressions;
    }

    Sequence sequence(ExpressionBlock e) {
        // Make predecessor table
        map<Expression, set<Expression> > pred =
            predecessors_from_expression_block(e);

        // maps expressions to the time they are scheduled in
        Sequence sequence;

        bool all_scheduled = false;
        while (!all_scheduled) {
            all_scheduled = true;

            // select expression with all predecessors scheduled and schedule it
            for (map<Expression, set<Expression> >::iterator it = pred.begin();
                it != pred.end(); ++it) {
                Expression expression = it->first;
                set<Expression> predecessors = it->second;

                // check if all predecessors have been scheduled
                bool all_pred_scheduled = true;
                Time max_pred_time = 0;
                for (set<Expression>::iterator pred = predecessors.begin();
                    pred != predecessors.end(); ++pred) {
                    if (sequence.find(*pred) == sequence.end()) {
                        all_scheduled = false;
                        all_pred_scheduled = false;
                    } else {
                        max_pred_time = max(max_pred_time,
                                        sequence.find(*pred)->second);
                    }
                }

                // if all predecessors are scheduled, this expression can be
                // scheduled
                if (all_pred_scheduled) {
                    sequence[expression] = max_pred_time + 1;
                }
            }
        }
        return sequence;
    }

    virtual vector<ExpressionBlock> operator()(ExpressionBlock e, SymbolTable) {
        return sequence_to_expression_blocks(sequence(e));
    }
};

map<Expression, set<Expression> > successors_from_expression_block(ExpressionBlock e) {
    map<Expression, set<Expression> > succ;
    for (vector<Expression>::iterator expression = e.begin();
        expression != e.end(); ++expression) {
        succ[*expression] = set<Expression>();
        for (vector<Expression>::iterator dep = expression + 1;
            dep != e.end(); ++dep) {

            // the result of this expression got assigned to again, no more
            // successors for this expression
            if (expression->result.name == dep->result.name) {
                break;
            }

            for (vector<Symbol>::iterator arg = dep->arguments.begin();
                arg != dep->arguments.end(); ++arg) {
                if (expression->result.name == arg->name) {
                    succ[*expression].insert(*dep);
                }
            }
        }
    }
    return succ;
}

vector<ExpressionBlock> latency_aware_sequence_to_expression_blocks(
    Sequence sequence, Time latency) {
    vector<ExpressionBlock> scheduled_expressions;
    for (Time t = latency; t > 0; t--) {
        ExpressionBlock e;
        for (Sequence::iterator it = sequence.begin();
            it != sequence.end(); ++it) {
            if (it->second == t) {
                e.add_expression(it->first);
            }
        }
        scheduled_expressions.push_back(e);
    }
    return vector<ExpressionBlock>(scheduled_expressions.rbegin(),
                                scheduled_expressions.rend());
}

class ALAPSchedule : public Schedule {
  public:
    Time latency;
    ALAPSchedule(Time latency) : latency(latency) {}

    // DIFFERENT FROM THE ONE THAT ASAP USES, LATENCY AWARE!

    Sequence sequence(ExpressionBlock e) {
        map<Expression, set<Expression> > succ =
            successors_from_expression_block(e);

        // maps expressions to the time they are scheduled in
        Sequence sequence;

        bool all_scheduled = false;
        while (!all_scheduled) {
            all_scheduled = true;

            // select expression with all successors scheduled and schedule it
            for (map<Expression, set<Expression> >::iterator it = succ.begin();
                it != succ.end(); ++it) {
                Expression expression = it->first;
                set<Expression> successors = it->second;

                // check if all successors have been scheduled
                bool all_succ_scheduled = true;
                Time min_succ_time = latency + 1;
                for (set<Expression>::iterator succ = successors.begin();
                    succ != successors.end(); ++succ) {
                    if (sequence.find(*succ) == sequence.end()) {
                        all_scheduled = false;
                        all_succ_scheduled = false;
                    } else {
                        min_succ_time = min(min_succ_time,
                                        sequence.find(*succ)->second);
                    }
                }

                true_or_die((min_succ_time - 1) >= 1,
                            "ALAP cannot meet latency requirements");

                // if all successors are scheduled, this expression can be
                // scheduled
                if (all_succ_scheduled) {
                    sequence[expression] = min_succ_time - 1;
                }
            }
        }
        return sequence;
    }

    virtual vector<ExpressionBlock> operator()(ExpressionBlock e, SymbolTable) {
        return latency_aware_sequence_to_expression_blocks(sequence(e), latency);
    }
};

typedef map<Expression, pair<Time, Time> > TimeFrames;
typedef enum {
    ERROR,
    MULTIPLIER,
    ADDER,
    LOGICAL,
} Hardware;

Hardware op_to_hw(TokenType t) {
    switch (t) {
        case ASSIGN:
        case LT:
        case GT:
        case EQ:
        case QMARK:
        case L_SHIFT:
        case R_SHIFT:
            return LOGICAL;
        case MULT:
            return MULTIPLIER;
        case PLUS:
        case MINUS:
            return ADDER;
        default:
            true_or_die(false,
            "unknown operator in op_to_hardware");
    }
    return ERROR;
}

// Minimum resource under latency constraint
class ForceDirectedSchedule : public Schedule {
  public:
    // resources: multiplier, adder/subtractor, logic/logical
    Time latency;
    ForceDirectedSchedule(Time latency) : latency(latency) {}

    // computes the self-force implicit in a particular scheduling decision
    float single_self_force(Expression exp, Time pt, Time start, Time end,
        map<Hardware, vector<float> > type_probabilities,
        map<Expression, vector<float> > operation_probabilities) {
        Hardware hw = op_to_hw(exp.op);
        float force = 0;
        for (Time t = start; t < end; t++) {
            float prob;
            if (pt == t) {
                prob = 1;
            } else {
                prob = 0;
            }

            // force[i] = distribution graph [i] * change in current
            // operations probability
            force += type_probabilities[hw][t] *
                        (prob - operation_probabilities[exp][t]);
        }
        return force;
    }

    // computes all possible self-forces for an unscheduled node
    vector<float> self_force(Expression exp, Time start, Time end,
        map<Hardware, vector<float> > type_probabilities,
        map<Expression, vector<float> > operation_probabilities) {
        vector<float> force;

        for (Time t = 1; t < start; t++) {
           force.push_back(FLT_MAX);
        }

        // pt -> possible time
        for (Time pt = start; pt <= end; pt++) {
            force.push_back(single_self_force(exp, pt, start, end,
                     type_probabilities, operation_probabilities));
        }

        return force;
    }

    bool all_scheduled(TimeFrames time_frames, Sequence sequence) {
        for (TimeFrames::iterator tfs = time_frames.begin();
            tfs != time_frames.end(); ++tfs) {
            Expression exp = tfs->first;
            if (sequence.find(exp) == sequence.end()) {
                return false;
            }
        }
        return true;
    }

    virtual vector<ExpressionBlock> operator()(ExpressionBlock e, SymbolTable) {
        // repeat {
            // compute the time frames; // timeframes are [ASAP, ALAP];
            // compute the operations and type probabilities
            // compute the self-forces, predecessor/successor forces and total forces;
            // schedule the operation with least force and update its time-frame;
        // } until (all operations scheduled);

        // compute the time frames
        ALAPSchedule alap(latency);
        Sequence alap_sequence = alap.sequence(e);

        ASAPSchedule asap;
        Sequence asap_sequence = asap.sequence(e);

        Sequence force_directed_sequence;

        TimeFrames time_frames;
        for (Sequence::iterator alap_it = alap_sequence.begin();
             alap_it != alap_sequence.end(); ++alap_it) {
            Expression exp = alap_it->first;
            Time alap_time = alap_it->second;
            Time asap_time = asap_sequence[exp];
            time_frames[exp] = make_pair<Time, Time>(asap_time, alap_time);
        }

        while (!all_scheduled(time_frames, force_directed_sequence)) {
            // compute the operations probabilities
            // equal to 1/framewidth inside timeframe, 0 outside
            map<Expression, vector<float> > operation_probabilities;
            for (int t = 1; t <= latency; t++) {
                for (TimeFrames::iterator tfs = time_frames.begin();
                    tfs != time_frames.end(); ++tfs) {
                    Expression exp = tfs->first;
                    Time start = tfs->second.first;
                    Time end = tfs->second.second;

                    if (operation_probabilities.find(exp)
                        == operation_probabilities.end()) {
                        operation_probabilities[exp] = vector<float>();
                    }

                    if (t >= start && t <= end) {
                        operation_probabilities[exp].push_back(1/(end - start + 1.0f));
                    } else {
                        operation_probabilities[exp].push_back(0);
                    }
                }
            }

            // compute the type probabilities (distribution graph for each type)
            map<Hardware, vector<float> > type_probabilities;
            for (int t = 1; t <= latency; t++) {

                // calculate all type probabilities at a given time
                map<Hardware, float> prob_t;
                for (map<Expression, vector<float> >::iterator op_prob =
                    operation_probabilities.begin(); op_prob !=
                    operation_probabilities.end(); ++op_prob) {
                    Expression exp = op_prob->first;
                    Hardware hw = op_to_hw(exp.op);
                    if (prob_t.find(hw) == prob_t.end()) {
                        prob_t[hw] = 0;
                    }
                    prob_t[hw] += operation_probabilities[exp][t];
                }

                // append the calculated type probabilities to the list of all
                // times
                for (map<Hardware, float>::iterator prob = prob_t.begin();
                    prob != prob_t.end(); ++prob) {
                    if (type_probabilities.find(prob->first)
                        == type_probabilities.end()) {
                        type_probabilities[prob->first] = vector<float>();
                    }

                    type_probabilities[prob->first].push_back(prob->second);
                }
            }

            // compute the self-forces, predecessors/successor forces and total forces
            // compute self-forces
            map<Expression, vector<float> > forces;
            // type_probabilities
            // operation_probabilities
            for (TimeFrames::iterator tfs = time_frames.begin();
                tfs != time_frames.end(); ++tfs) {
                Expression exp = tfs->first;
                Time start = tfs->second.first;
                Time end = tfs->second.second;
                forces[exp] = self_force(exp, start, end,
                        type_probabilities, operation_probabilities);
            }

            // lookup predecessor time frames and check if scheduling this
            // operation at some time will force a predecessor to be scheduled at
            // a particular time (shrinks the timeframe to one value)

            // predecessor forces
            map<Expression, set<Expression> > predecessors = predecessors_from_expression_block(e);
            for (TimeFrames::iterator tfs = time_frames.begin();
                tfs != time_frames.end(); ++tfs) {
                Expression exp = tfs->first;
                Time start = tfs->second.first;
                Time end = tfs->second.second;
                for (Time t = start; t <= end; t++) {
                    for (set<Expression>::iterator pred = predecessors[exp].begin(); pred !=
                        predecessors[exp].end(); ++pred) {
                        Time pred_start = time_frames[*pred].first;
                        Time pred_end = start - 1;
                        forces[exp][t] += single_self_force(*pred, t, pred_start, pred_end,
                                type_probabilities, operation_probabilities);
                    }
                }
            }

            // successor forces
            map<Expression, set<Expression> > successors =
                successors_from_expression_block(e);
            for (TimeFrames::iterator tfs = time_frames.begin();
                tfs != time_frames.end(); ++tfs) {
                Expression exp = tfs->first;
                Time start = tfs->second.first;
                Time end = tfs->second.second;
                for (Time t = start; t <= end; t++) {
                    for (set<Expression>::iterator succ = successors[exp].begin(); succ !=
                        successors[exp].end(); ++succ) {
                        Time succ_start = end + 1;
                        Time succ_end = time_frames[*succ].second;
                        forces[exp][t] += single_self_force(*succ, t, succ_start, succ_end,
                                type_probabilities, operation_probabilities);
                    }
                }
            }

            // schedule the operation with least force and update its time-frame
            float min_force = FLT_MAX;
            Expression to_schedule;
            Time scheduled_time;
            for (map<Expression, vector<float> >::iterator it = forces.begin();
                it != forces.end(); ++it) {
                Expression exp = it->first;
                vector<float> force = it->second;
                for (Time t = time_frames[exp].first; t <= time_frames[exp].second; ++t) {
                    if (force[t] < min_force &&
                        force_directed_sequence.find(exp)
                        == force_directed_sequence.end()) {
                        min_force = force[t];
                        to_schedule = exp;
                        scheduled_time = t;
                    }
                }
            }

            // lock down the time frame
            time_frames[to_schedule] = make_pair<Time, Time>(scheduled_time, scheduled_time);

            // update predecessors and successors
            for (set<Expression>::iterator pred = predecessors[to_schedule].begin(); pred !=
                predecessors[to_schedule].end(); ++pred) {
                time_frames[*pred].second = scheduled_time - 1;
            }
            for (set<Expression>::iterator succ = successors[to_schedule].begin(); succ !=
                    successors[to_schedule].end(); ++succ) {
                time_frames[*succ].first = scheduled_time + 1;
            }

            // schedule everything with a frame width of one (only one place to schedule)
            for (TimeFrames::iterator tfs = time_frames.begin();
                tfs != time_frames.end(); ++tfs) {
                Expression exp = tfs->first;
                Time start = tfs->second.first;
                Time end = tfs->second.second;
                if (start == end) {
                    force_directed_sequence[exp] = start;
                }
            }
        }

        return latency_aware_sequence_to_expression_blocks(
            force_directed_sequence, latency);
    }
};

class Program {
public:
    Program(SymbolTable symbols, Statements* statements) :
        symbols(symbols),
        statements(statements) {}

    void generate_fsm(ostream& os, Schedule* schedule) {
        os << "`timescale 1ns / 1ps\n";
        os << "module HLSM(Clk, Rst, Start, Done";
        for (SymbolTable::iterator sym = symbols.begin();
             sym != symbols.end(); ++sym) {
            if (sym->second.type == INPUT || sym->second.type == OUTPUT) {
                os << ", " << sym->first;
            }
        }
        os << ");\n";
        os << "input Clk, Rst, Start;\n";
        os << "output reg Done;\n";
        os << "reg[31:0] State;\n";
        for (SymbolTable::iterator sym = symbols.begin();
             sym != symbols.end(); ++sym) {
            if (sym->second.type == INPUT) {
                os << "input [31:0] " << sym->first << ";\n";
            } else if (sym->second.type == OUTPUT) {
                os << "output reg [31:0] " << sym->first << ";\n";
            } else if (sym->second.type == REG) {
                os << "reg [31:0] " << sym->first << ";\n";
            } else {
                os << "// NONE -> " << sym->first << ";\n";
            }
        }

        state start = unique_state();
        os << "\nalways @(posedge Clk) begin\n";
        os << "    if (Rst == 1) begin\n";
        os << "        State <= " << start << ";\n";
        for (SymbolTable::iterator sym = symbols.begin();
             sym != symbols.end(); ++sym) {
            if (sym->second.type == OUTPUT || sym->second.type == REG) {
                os << "        " << sym->first << " <= 0;\n";
            }
        }

        state first = unique_state();
        os << "    end else begin\n";
        os << "        case (State)\n";
        os << "        // WAIT STATE\n";
        os << "        " << start << ": begin\n";
        os << "            if (Start) begin\n";
        os << "                State <= " << first << ";\n";
        os << "                Done <= 0;\n";
        os << "            end else begin\n";
        os << "                State <= " << start << ";\n";
        os << "            end\n";
        os << "        end\n";

        // Easier to schedule if the input and output variables are known, so
        // pass symbol table to generate_fsm
        state end_state = unique_end_state();
        statements->generate_fsm(symbols, os, schedule, first, end_state);

        os << "        // FINAL STATE\n";
        os << "        " << end_state << ": begin\n";
        os << "            Done <= 1;\n";
        os << "            State <= " << start << ";\n";
        os << "        end\n";
        os << "        endcase\n";
        os << "    end\n";
        os << "end\n";
        os << "\nendmodule\n";
    }

    ~Program() {
        delete statements;
    }

    SymbolTable symbols;
    Statements* statements;
};

class Parser {
  public:
    Parser(istream& in) : t(in) {};

    Program program() {
        declarations();
        return Program(s, statements());
    }

  private:
    class SymtableInsertion {
      public:
        Parser* p;
        TokenType token_type;
        SymbolType symbol_type;
        SymtableInsertion(Parser* p, TokenType t, SymbolType s) :
            p(p), token_type(t), symbol_type(s) {}

        bool operator()(TokenStream& t, SymbolTable& s) {
            p->ignore_newlines();
            if (t->type == token_type) {
                t++;
            } else {
                return false;
            }

            p->accept(COLON, ":");

            while (t->type == VAR) {
                true_or_die(s.find(t->contents) == s.end(),
                        "attempting to redefine variable " + t-> contents);
                s[t->contents] = Symbol(t->contents, symbol_type);
                t++;
            }
            return true;
        }
    };

    void declarations() {
        // Declarations can come in any order, compensate with this by iterating
        // over a list of declarations we haven't seen yet.
        vector<SymtableInsertion> operations;
        operations.push_back(SymtableInsertion(this, INPUTS, INPUT));
        operations.push_back(SymtableInsertion(this, OUTPUTS, OUTPUT));
        operations.push_back(SymtableInsertion(this, REGS, REG));
        vector<SymtableInsertion> remaining;

        int num_ops = operations.size();
        for (int i = 0; i < num_ops; ++i) {
            for (vector<SymtableInsertion>::iterator op = operations.begin();
                op != operations.end(); ++op) {
                if (!(*op)(t, s)) {
                    remaining.push_back(*op);
                }
            }
            operations = remaining;
            remaining.clear();
        }
    }

    Statements* statements() {
        Statements* statements = new Statements();
        while (true) {
            ignore_newlines();
            if (t->type == VAR) {
                statements->append(expressions());
            } else if (t->type == WHILE) {
                statements->append(while_());
            } else if (t->type == DO) {
                statements->append(do_());
            } else if (t->type == IF) {
                statements->append(if_());
            } else {
                break;
            }
        }
        true_or_die(t->type == EOF_TOKEN || t->type == R_CURLY,
            "invalid characters \"" + t->contents +
            "\" found in statement block");
        return statements;
    }

    Symbol accept_variable(SymbolType direction) {
        true_or_die(direction == INPUT || direction == OUTPUT,
            "variable direction must be INPUT or OUTPUT");
        SymbolTable::iterator symbol = s.find(t->contents);
        true_or_die(symbol != s.end(), "symbol " + t->contents + " not found");
        true_or_die(symbol->second.type == direction ||
                    symbol->second.type == REG,
            "attempting " +
            string(direction == OUTPUT ? "assignment to" : "reading from a")
            + " variable \"" + symbol->first + "\" which is not an " +
            string(direction == OUTPUT ? "OUTPUT" : "INPUT")
            + " or REG");
        t++;
        return symbol->second;
    }

    void ignore_newlines() {
        while (t->type == NEWLINE) {
            t++;
        }
    }

    void accept(TokenType type, string name) {
        true_or_die(t->type == type, "expected '" + name + "' but found " + t->contents);
        t++;
    }

    // find series of expressions, push them into an expression block for
    // scheduling
    Statements* expressions() {
        ExpressionBlock* block = new ExpressionBlock();
        while (t->type == VAR) {
            // result
            Expression e;
            e.result = accept_variable(OUTPUT);

            accept(ASSIGN, "assignment");

            // first arg
            e.add_argument(accept_variable(INPUT));

            // Just an assignment
            if (t->type == VAR
             || t->type == IF
             || t->type == WHILE
             || t->type == DO
             || t->type == EOF_TOKEN
             || t->type == NEWLINE
             || t->type == R_CURLY) {
                e.op = ASSIGN;
                block->add_expression(e);
                continue;
            }

            // operator
            true_or_die(t->is_op(), "non-operator " + t->contents +
                    " in expression");
            e.op = t->type;
            t++;

            // second arg
            e.add_argument(accept_variable(INPUT));

            // ternary operator
            if (e.op == QMARK && t->type == COLON) {
                t++;
                // third arg
                e.add_argument(accept_variable(INPUT));
            }
            block->add_expression(e);
        }
        return block;
    }

    Statements* while_() {
        While* w = new While();
        accept(WHILE, "while");
        accept(L_PAREN, "(");

        w->condition = accept_variable(INPUT);

        accept(R_PAREN, ")");
        accept(L_CURLY, "{");

        w->append(statements());

        accept(R_CURLY, "}");
        return w;
    }

    Statements* do_() {
        Do* d = new Do();
        accept(DO, "do");
        accept(L_CURLY, "{");

        d->append(statements());

        accept(R_CURLY, "}");

        accept(WHILE, "while");
        accept(L_PAREN, "(");

        d->condition = accept_variable(INPUT);

        accept(R_PAREN, ")");
        return d;
    }

    Statements* if_() {
        If* i = new If();
        accept(IF, "if");
        accept(L_PAREN, "(");

        i->condition = accept_variable(INPUT);

        accept(R_PAREN, ")");
        accept(L_CURLY, "{");

        i->append(statements());

        accept(R_CURLY, "}");
        return i;
    }

    TokenStream t;
    SymbolTable s;
};

void usage(char** argv) {
    cerr << '\n' << "Usage: " << argv[0] <<
        " -ns cfile verilogfile OR -fd cfile verilogfile latency\n\n";
    exit(-1);
}

int main(int argc, char** argv) {
    if (argc != 4 && argc != 5) {
        usage(argv);
    }

    Schedule* schedule;
    if (string(argv[1]) == "-ns") {
        schedule = new NoSchedule();
    } else if (argc == 5 && string(argv[1]) == "-fd") {
        // TODO make this actually check things
        schedule = new ForceDirectedSchedule(atoi(argv[4]));
    } else {
        usage(argv);
    }

    /*
    delete schedule;
    schedule = new ASAPSchedule();
    schedule = new ALAPSchedule(6);
    */

    string verilog_name(argv[3]);
    fstream c_file(argv[2]);
    true_or_die(c_file.is_open(), "the c file could not be opened");

    Parser p(c_file);
    Program program = p.program();
    c_file.close();

    program.generate_fsm(cout, schedule);
    delete schedule;

    return 0;
}

