# image-generator Skill

## Purpose
Create concept diagrams, architecture illustrations, and visual aids using Mermaid and guidelines for external tools.

## Key Patterns

### Mermaid Diagram Types

**1. Architecture Diagrams**
```mermaid
graph TD
    A[ROS 2 Application] --> B[rclpy/rclcpp Client Library]
    B --> C[DDS Middleware]
    C --> D[Network Stack]
    D --> E[Other Nodes]

    style A fill:#e1f5ff
    style C fill:#fff9c4
    style E fill:#f3e5f5
```

**2. Sequence Diagrams**
```mermaid
sequenceDiagram
    participant Client
    participant Service
    participant Server

    Client->>Service: Send request
    Service->>Server: Forward request
    Server-->>Service: Send response
    Service-->>Client: Return response
```

**3. Flow Charts**
```mermaid
flowchart LR
    Start --> Condition{Check?}
    Condition -->|Yes| ActionA[Do A]
    Condition -->|No| ActionB[Do B]
    ActionA --> End
    ActionB --> End
```

**4. Class Diagrams**
```mermaid
classDiagram
    class Node {
        +String name
        +create_publisher()
        +create_subscriber()
    }
    class Publisher {
        +publish()
    }
    Node --> Publisher
```

### Visual Guidelines
- Use consistent colors: Blue for app layer, Yellow for middleware, Purple for external
- Keep diagrams simple: Max 7-8 nodes per diagram
- Add legends for complex diagrams
- Use arrows to show data flow direction

### External Tool Recommendations
- **Excalidraw**: Hand-drawn style diagrams
- **Figma**: Professional UI mockups
- **Draw.io**: Complex system architectures
- **PlantUML**: Automated diagram generation

## Usage Context
- Explaining system architecture
- Visualizing data flow
- Illustrating algorithms
- Concept clarification
