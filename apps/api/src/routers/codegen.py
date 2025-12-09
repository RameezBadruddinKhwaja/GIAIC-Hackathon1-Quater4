"""
Code Generation Router - ROS 2 Text-to-Code Generator

Converts natural language descriptions to ROS 2 Python/C++ code using Gemini.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Literal
import logging

from ..services.gemini_client import get_gemini_client, CHAT_MODEL

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/codegen")

class CodeGenRequest(BaseModel):
    description: str
    language: Literal["python", "cpp"] = "python"
    node_type: str = "publisher"  # publisher, subscriber, service, action, etc.

class CodeGenResponse(BaseModel):
    code: str
    language: str
    description: str
    explanation: str

@router.post("/", response_model=CodeGenResponse)
async def generate_ros2_code(request: CodeGenRequest):
    """
    Generate ROS 2 code from natural language description.

    Args:
        request: CodeGenRequest with description and language preference

    Returns:
        CodeGenResponse with generated code and explanation
    """
    try:
        # Validate input
        description = request.description.strip()

        if not description:
            raise HTTPException(status_code=400, detail="Description cannot be empty")

        if len(description) > 1000:
            raise HTTPException(status_code=400, detail="Description too long (max 1000 chars)")

        # Build system prompt based on language
        if request.language == "python":
            system_prompt = generate_python_system_prompt()
        else:
            system_prompt = generate_cpp_system_prompt()

        # Call Gemini
        client = get_gemini_client()

        response = client.chat.completions.create(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Generate ROS 2 {request.language} code for: {description}"}
            ],
            temperature=0.3,  # Lower temperature for consistent code
            max_tokens=1500
        )

        full_response = response.choices[0].message.content

        # Parse response (expected format: code block + explanation)
        code, explanation = parse_code_response(full_response, request.language)

        return CodeGenResponse(
            code=code,
            language=request.language,
            description=description,
            explanation=explanation
        )

    except Exception as e:
        logger.error(f"Error in code generation: {e}")
        raise HTTPException(status_code=500, detail=str(e))

def generate_python_system_prompt() -> str:
    """Generate system prompt for Python code generation"""
    return """You are an expert ROS 2 Python code generator.

**Rules:**
1. Generate ONLY valid ROS 2 Humble Python code
2. Use rclpy best practices
3. Include proper imports
4. Add docstrings and comments
5. Handle errors gracefully
6. Follow PEP 8 style guide

**Template Structure:**
```python
import rclpy
from rclpy.node import Node
# ... other imports

class YourNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # ... initialization

    # ... methods

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Output Format:**
1. First, provide the complete code in a markdown code block
2. Then, provide a brief explanation (2-3 sentences)

Example response:
```python
[code here]
```

**Explanation:** This node creates a publisher that... [explanation]
"""

def generate_cpp_system_prompt() -> str:
    """Generate system prompt for C++ code generation"""
    return """You are an expert ROS 2 C++ code generator.

**Rules:**
1. Generate ONLY valid ROS 2 Humble C++ code
2. Use rclcpp best practices
3. Include proper headers
4. Add Doxygen-style comments
5. Use modern C++17 features
6. Follow ROS 2 naming conventions

**Template Structure:**
```cpp
#include "rclcpp/rclcpp.hpp"
// ... other includes

class YourNode : public rclcpp::Node
{
public:
    YourNode() : Node("node_name")
    {
        // ... initialization
    }

private:
    // ... members
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YourNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Output Format:**
1. First, provide the complete code in a markdown code block
2. Then, provide a brief explanation (2-3 sentences)

Example response:
```cpp
[code here]
```

**Explanation:** This node creates a publisher that... [explanation]
"""

def parse_code_response(response: str, language: str) -> tuple[str, str]:
    """
    Parse Gemini response to extract code and explanation.

    Args:
        response: Raw Gemini response
        language: Programming language (python or cpp)

    Returns:
        Tuple of (code, explanation)
    """
    import re

    # Extract code from markdown code block
    if language == "python":
        code_pattern = r'```python\n(.*?)\n```'
    else:
        code_pattern = r'```cpp\n(.*?)\n```'

    code_match = re.search(code_pattern, response, re.DOTALL)

    if code_match:
        code = code_match.group(1).strip()
    else:
        # Fallback: try generic code block
        generic_pattern = r'```\n(.*?)\n```'
        generic_match = re.search(generic_pattern, response, re.DOTALL)
        if generic_match:
            code = generic_match.group(1).strip()
        else:
            code = response.strip()

    # Extract explanation
    explanation_pattern = r'\*\*Explanation:\*\*\s*(.*?)(?:\n\n|$)'
    explanation_match = re.search(explanation_pattern, response, re.DOTALL)

    if explanation_match:
        explanation = explanation_match.group(1).strip()
    else:
        # Fallback: use text after code block
        explanation_parts = response.split('```')
        if len(explanation_parts) > 2:
            explanation = explanation_parts[2].strip()
        else:
            explanation = "Code generated successfully."

    return code, explanation

# Additional endpoint for code validation
@router.post("/validate")
async def validate_ros2_code(code: str, language: Literal["python", "cpp"]):
    """
    Validate ROS 2 code syntax (basic validation).

    Args:
        code: ROS 2 code to validate
        language: Programming language

    Returns:
        Validation results
    """
    try:
        errors = []
        warnings = []

        if language == "python":
            # Check for required imports
            if "import rclpy" not in code:
                errors.append("Missing 'import rclpy'")
            if "from rclpy.node import Node" not in code:
                errors.append("Missing 'from rclpy.node import Node'")

            # Check for main function
            if "def main(args=None):" not in code:
                warnings.append("Missing 'main' function")

        else:  # C++
            # Check for required includes
            if "#include \"rclcpp/rclcpp.hpp\"" not in code:
                errors.append("Missing '#include \"rclcpp/rclcpp.hpp\"'")

            # Check for main function
            if "int main(int argc, char **argv)" not in code:
                errors.append("Missing 'main' function")

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings
        }

    except Exception as e:
        logger.error(f"Error validating code: {e}")
        raise HTTPException(status_code=500, detail=str(e))
