#include "metrics.hpp"
#include <functional>
#include <memory>
#include <sstream>
namespace fourwis {
namespace {
struct Value {
  std::vector<double> x;
  bool scalar = true;
  explicit Value(double n = 0) : x{n} {}
  Value(std::vector<double> v, bool s = false) : x(std::move(v)), scalar(s) {}
  double number() const {
    if (!scalar || x.size() != 1)
      throw Error(422, "expression requires a scalar");
    return x[0];
  }
};
struct Node {
  std::string op;
  double number = 0;
  std::vector<std::shared_ptr<Node>> children;
};
using N = std::shared_ptr<Node>;
struct Parser {
  std::string source, token;
  double number = 0;
  size_t pos = 0, count = 0;
  const Run &run;
  Parser(std::string text, const Run &r) : source(std::move(text)), run(r) {
    if (source.size() > 8192)
      throw Error(422, "metric expression too long");
    next();
  }
  void next() {
    while (pos < source.size() &&
           std::isspace(static_cast<unsigned char>(source[pos])))
      pos++;
    if (pos == source.size()) {
      token = "";
      return;
    }
    char c = source[pos];
    if (std::isdigit(static_cast<unsigned char>(c)) ||
        (c == '.' && pos + 1 < source.size() &&
         std::isdigit(static_cast<unsigned char>(source[pos + 1])))) {
      char *end = nullptr;
      number = std::strtod(source.c_str() + pos, &end);
      pos = size_t(end - source.c_str());
      token = "#";
      return;
    }
    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      size_t start = pos++;
      while (pos < source.size() &&
             (std::isalnum(static_cast<unsigned char>(source[pos])) ||
              source[pos] == '_'))
        pos++;
      token = source.substr(start, pos - start);
      return;
    }
    token = source.substr(pos++, 1);
    if (pos < source.size() &&
        std::set<std::string>{"**", "//", "<=", ">=", "==", "!="}.count(
            token + source[pos]))
      token += source[pos++];
  }
  N node(std::string op, std::vector<N> kids = {}) {
    if (++count > 2048)
      throw Error(422, "metric expression too complex");
    auto n = std::make_shared<Node>();
    n->op = std::move(op);
    n->children = std::move(kids);
    return n;
  }
  void require(const std::string &expected) {
    if (token != expected)
      throw Error(422, "invalid metric expression: expected " + expected);
    next();
  }
  N primary(int depth) {
    if (depth > 96)
      throw Error(422, "metric nesting too deep");
    if (token == "#") {
      auto n = node("number");
      n->number = number;
      next();
      return n;
    }
    if (token == "(" || token == "[") {
      bool list = token == "[";
      next();
      std::vector<N> items;
      if (token != (list ? "]" : ")")) {
        items.push_back(conditional(depth + 1));
        while (token == ",") {
          next();
          if (token == (list ? "]" : ")"))
            break;
          items.push_back(conditional(depth + 1));
        }
      }
      require(list ? "]" : ")");
      if (!list && items.size() == 1)
        return items[0];
      return node("list", items);
    }
    auto name = token;
    if (name.empty() ||
        !(std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_'))
      throw Error(422, "invalid metric expression token");
    next();
    if (token == "(") {
      static const std::set<std::string> allowed = {
          "abs",   "min",  "max",   "mean", "median", "sum",     "sqrt",
          "atan2", "atan", "sin",   "cos",  "tan",    "degrees", "radians",
          "clip",  "sign", "first", "last", "peak",   "rms",     "steady"};
      if (!allowed.count(name))
        throw Error(422, "unknown metric function: " + name);
      next();
      std::vector<N> args;
      if (token != ")") {
        args.push_back(conditional(depth + 1));
        while (token == ",") {
          next();
          args.push_back(conditional(depth + 1));
        }
      }
      require(")");
      size_t lower = name == "clip"    ? 3
                     : name == "atan2" ? 2
                                       : 1,
             upper = name == "steady" ? 2 : lower;
      if (args.size() < lower || args.size() > upper)
        throw Error(422, "invalid metric function argument count");
      return node("call:" + name, args);
    }
    if (name != "pi" && name != "g" && name != "True" && name != "False" &&
        name != "t" && !run.channels.count(name))
      throw Error(422, "unknown metric channel: " + name);
    return node("name:" + name);
  }
  N unary(int depth) {
    if (token == "+" || token == "-") {
      auto op = token;
      next();
      return node("unary" + op, {unary(depth + 1)});
    }
    auto a = primary(depth + 1);
    if (token == "**") {
      next();
      a = node("**", {a, unary(depth + 1)});
    }
    return a;
  }
  N mul(int d) {
    auto a = unary(d);
    while (token == "*" || token == "/" || token == "//" || token == "%") {
      auto op = token;
      next();
      a = node(op, {a, unary(d)});
    }
    return a;
  }
  N add(int d) {
    auto a = mul(d);
    while (token == "+" || token == "-") {
      auto op = token;
      next();
      a = node(op, {a, mul(d)});
    }
    return a;
  }
  N compare(int d) {
    auto a = add(d);
    std::vector<N> parts;
    while (
        std::set<std::string>{"<", ">", "<=", ">=", "==", "!="}.count(token)) {
      auto op = token;
      next();
      auto b = add(d);
      parts.push_back(node(op, {a, b}));
      a = b;
    }
    if (parts.empty())
      return a;
    for (size_t i = 1; i < parts.size(); i++)
      parts[0] = node("and", {parts[0], parts[i]});
    return parts[0];
  }
  N neg(int d) {
    if (token == "not") {
      next();
      return node("not", {neg(d + 1)});
    }
    return compare(d);
  }
  N logic(int d) {
    auto a = neg(d);
    while (token == "and") {
      next();
      a = node("and", {a, neg(d)});
    }
    return a;
  }
  N disjunction(int d) {
    auto a = logic(d);
    while (token == "or") {
      next();
      a = node("or", {a, logic(d)});
    }
    return a;
  }
  N conditional(int d = 0) {
    auto a = disjunction(d);
    if (token == "if") {
      next();
      auto test = disjunction(d);
      require("else");
      return node("if", {test, a, conditional(d + 1)});
    }
    return a;
  }
};
Value element(const Value &a, const Value &b,
              const std::function<double(double, double)> &fn) {
  size_t n = a.scalar ? b.x.size() : a.x.size();
  if (!a.scalar && !b.scalar && a.x.size() != b.x.size())
    throw Error(422, "metric array lengths disagree");
  std::vector<double> out;
  out.reserve(n);
  for (size_t i = 0; i < n; i++)
    out.push_back(fn(a.x[a.scalar ? 0 : i], b.x[b.scalar ? 0 : i]));
  return Value(out, a.scalar && b.scalar);
}
Value evaluate(const N &n, const Run &r) {
  auto op = n->op;
  if (op == "number")
    return Value(n->number);
  if (op.rfind("name:", 0) == 0) {
    auto name = op.substr(5);
    if (name == "pi")
      return Value(pi);
    if (name == "g")
      return Value(9.80665);
    if (name == "True" || name == "False")
      return Value(name == "True" ? 1. : 0.);
    if (name == "t")
      return Value(r.t);
    return Value(r.channels.at(name));
  }
  if (op == "if")
    return evaluate(
        n->children[evaluate(n->children[0], r).number() != 0 ? 1 : 2], r);
  if (op == "and" || op == "or") {
    auto a = evaluate(n->children[0], r);
    bool truth = a.number() != 0;
    if ((op == "and" && !truth) || (op == "or" && truth))
      return a;
    return evaluate(n->children[1], r);
  }
  if (op == "list") {
    std::vector<double> out;
    for (auto &c : n->children) {
      auto v = evaluate(c, r);
      if (!v.scalar)
        throw Error(422, "nested metric arrays are unsupported");
      out.push_back(v.number());
    }
    return Value(out);
  }
  std::vector<Value> args;
  for (auto &c : n->children)
    args.push_back(evaluate(c, r));
  auto unary = [&](const std::function<double(double)> &f) {
    auto out = args[0];
    for (auto &x : out.x)
      x = f(x);
    return out;
  };
  if (op == "unary+")
    return args[0];
  if (op == "unary-")
    return unary([](double x) { return -x; });
  if (op == "not")
    return Value(args[0].number() == 0 ? 1. : 0.);
  if (op.rfind("call:", 0) == 0) {
    auto f = op.substr(5);
    if (f == "abs")
      return unary([](double x) { return std::abs(x); });
    if (f == "sqrt")
      return unary([](double x) { return std::sqrt(x); });
    if (f == "sin")
      return unary([](double x) { return std::sin(x); });
    if (f == "cos")
      return unary([](double x) { return std::cos(x); });
    if (f == "tan")
      return unary([](double x) { return std::tan(x); });
    if (f == "atan")
      return unary([](double x) { return std::atan(x); });
    if (f == "degrees")
      return unary([](double x) { return deg(x); });
    if (f == "radians")
      return unary([](double x) { return rad(x); });
    if (f == "sign")
      return unary([](double x) { return std::isnan(x) ? nan : sign(x); });
    if (f == "atan2")
      return element(args[0], args[1],
                     [](double a, double b) { return std::atan2(a, b); });
    if (f == "clip")
      return element(element(args[0], args[1],
                             [](double a, double b) { return std::max(a, b); }),
                     args[2],
                     [](double a, double b) { return std::min(a, b); });
    auto x = args[0].x;
    if (f == "first" || f == "last") {
      if (x.empty())
        throw Error(422, "empty metric channel");
      return Value(f == "first" ? x.front() : x.back());
    }
    if (f == "steady") {
      double frac = args.size() > 1 ? args[1].number() : .2;
      size_t count = std::min(
          x.size(),
          size_t(std::max(1., std::nearbyint(x.size() * clip(frac, .001, 1)))));
      if (count)
        x.erase(x.begin(), x.end() - count);
    }
    std::vector<double> finite;
    for (double v : x)
      if (!std::isnan(v))
        finite.push_back(v);
    if (finite.empty())
      return Value(f == "sum" ? 0. : nan);
    if (f == "min")
      return Value(*std::min_element(finite.begin(), finite.end()));
    if (f == "max")
      return Value(*std::max_element(finite.begin(), finite.end()));
    if (f == "peak") {
      double v = 0;
      for (double x : finite)
        v = std::max(v, std::abs(x));
      return Value(v);
    }
    if (f == "median") {
      std::sort(finite.begin(), finite.end());
      size_t m = finite.size() / 2;
      return Value(finite.size() % 2 ? finite[m]
                                     : (finite[m - 1] + finite[m]) / 2);
    }
    double sum = 0;
    for (double v : finite)
      sum += f == "rms" ? v * v : v;
    if (f == "sum")
      return Value(sum);
    return Value(f == "rms" ? std::sqrt(sum / finite.size())
                            : sum / finite.size());
  }
  return element(args[0], args[1], [&](double a, double b) {
    if (op == "+")
      return a + b;
    if (op == "-")
      return a - b;
    if (op == "*")
      return a * b;
    if (op == "/")
      return a / b;
    if (op == "//")
      return std::floor(a / b);
    if (op == "%")
      return a - b * std::floor(a / b);
    if (op == "**")
      return std::pow(a, b);
    if (op == "<")
      return double(a < b);
    if (op == "<=")
      return double(a <= b);
    if (op == ">")
      return double(a > b);
    if (op == ">=")
      return double(a >= b);
    if (op == "==")
      return double(a == b);
    if (op == "!=")
      return double(a != b);
    throw Error(422, "unknown metric operator");
  });
}
} // namespace
double expression(const std::string &source, const Run &r, bool validate_only) {
  Parser parser(source, r);
  auto tree = parser.conditional();
  if (!parser.token.empty())
    throw Error(422, "unexpected expression token: " + parser.token);
  return validate_only ? 0 : evaluate(tree, r).number();
}
} // namespace fourwis
