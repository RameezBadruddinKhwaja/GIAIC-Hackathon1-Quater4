#!/usr/bin/env python3
"""
Automated System Verification Script

Tests:
1. Backend startup and health check
2. Frontend build verification
3. Environment variables validation
"""

import os
import sys
import time
import subprocess
import signal
import httpx
from pathlib import Path

# ANSI color codes
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'

def print_header(text):
    print(f"\n{BLUE}{'=' * 60}{RESET}")
    print(f"{BLUE}{text}{RESET}")
    print(f"{BLUE}{'=' * 60}{RESET}\n")

def print_success(text):
    print(f"{GREEN}✅ {text}{RESET}")

def print_error(text):
    print(f"{RED}❌ {text}{RESET}")

def print_info(text):
    print(f"{YELLOW}ℹ️  {text}{RESET}")

class BackendProcess:
    def __init__(self):
        self.process = None

    def start(self):
        """Start the backend server in background"""
        print_info("Starting backend server...")

        # Change to apps/api directory
        api_dir = Path(__file__).parent.parent

        # Start uvicorn in background
        self.process = subprocess.Popen(
            ["uvicorn", "src.main:app", "--port", "8000"],
            cwd=api_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )

        print_info(f"Backend PID: {self.process.pid}")

    def wait_for_health(self, max_retries=30, delay=1):
        """Poll /api/health until 200 OK"""
        print_info("Polling /api/health endpoint...")

        for i in range(max_retries):
            try:
                response = httpx.get("http://localhost:8000/api/health", timeout=5.0)
                if response.status_code == 200:
                    print_success(f"Backend healthy! (attempt {i+1}/{max_retries})")
                    print_info(f"Health check response: {response.json()}")
                    return True
            except (httpx.ConnectError, httpx.TimeoutException):
                pass

            time.sleep(delay)
            print(f"  Waiting... ({i+1}/{max_retries})", end='\r')

        print_error(f"Backend failed to become healthy after {max_retries} seconds")
        return False

    def stop(self):
        """Stop the backend server"""
        if self.process:
            print_info("Stopping backend server...")
            try:
                # Kill entire process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
                print_success("Backend stopped")
            except Exception as e:
                print_error(f"Error stopping backend: {e}")

def verify_environment():
    """Check required environment variables"""
    print_header("Step 1: Environment Variables Verification")

    required_vars = [
        "GEMINI_API_KEY",
        "NEON_CONNECTION_STRING",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "JWT_SECRET"
    ]

    missing_vars = []

    for var in required_vars:
        value = os.getenv(var)
        if value:
            # Mask the value for security
            masked = value[:8] + "..." if len(value) > 8 else "***"
            print_success(f"{var}: {masked}")
        else:
            print_error(f"{var}: NOT SET")
            missing_vars.append(var)

    if missing_vars:
        print_error(f"Missing environment variables: {', '.join(missing_vars)}")
        return False

    print_success("All environment variables configured!")
    return True

def verify_backend():
    """Start backend and verify health check"""
    print_header("Step 2: Backend Verification")

    backend = BackendProcess()

    try:
        backend.start()

        if not backend.wait_for_health():
            return False

        # Test a few more endpoints
        try:
            # Test root endpoint
            response = httpx.get("http://localhost:8000/", timeout=5.0)
            if response.status_code == 200:
                print_success("Root endpoint (/) working")
            else:
                print_error(f"Root endpoint returned {response.status_code}")

            # Test API docs
            response = httpx.get("http://localhost:8000/docs", timeout=5.0)
            if response.status_code == 200:
                print_success("API docs (/docs) accessible")
            else:
                print_error(f"API docs returned {response.status_code}")

        except Exception as e:
            print_error(f"Error testing endpoints: {e}")
            return False

        return True

    finally:
        backend.stop()

def verify_frontend():
    """Build frontend to check for errors"""
    print_header("Step 3: Frontend Build Verification")

    # Change to apps/docs directory
    docs_dir = Path(__file__).parent.parent.parent / "docs"

    print_info("Running npm run build...")

    try:
        result = subprocess.run(
            ["npm", "run", "build"],
            cwd=docs_dir,
            capture_output=True,
            text=True,
            timeout=300  # 5 minute timeout
        )

        if result.returncode == 0:
            print_success("Frontend build successful!")
            return True
        else:
            print_error("Frontend build failed!")
            print(f"{RED}STDOUT:{RESET}")
            print(result.stdout)
            print(f"{RED}STDERR:{RESET}")
            print(result.stderr)
            return False

    except subprocess.TimeoutExpired:
        print_error("Frontend build timed out after 5 minutes")
        return False
    except Exception as e:
        print_error(f"Error building frontend: {e}")
        return False

def main():
    """Run all verification steps"""
    print_header("🔍 AUTOMATED SYSTEM VERIFICATION")

    # Load .env file
    from dotenv import load_dotenv
    env_path = Path(__file__).parent.parent / ".env"
    load_dotenv(env_path)

    results = {
        "environment": False,
        "backend": False,
        "frontend": False
    }

    # Step 1: Environment variables
    results["environment"] = verify_environment()

    if not results["environment"]:
        print_error("Environment verification failed. Fix .env file first.")
        sys.exit(1)

    # Step 2: Backend
    results["backend"] = verify_backend()

    if not results["backend"]:
        print_error("Backend verification failed.")
        sys.exit(1)

    # Step 3: Frontend
    results["frontend"] = verify_frontend()

    # Final summary
    print_header("📊 VERIFICATION SUMMARY")

    for component, passed in results.items():
        status = f"{GREEN}✅ PASS{RESET}" if passed else f"{RED}❌ FAIL{RESET}"
        print(f"{component.upper():20s} {status}")

    # Overall status
    print()
    if all(results.values()):
        print(f"{GREEN}{'=' * 60}{RESET}")
        print(f"{GREEN}✅ SYSTEM READY - ALL CHECKS PASSED!{RESET}")
        print(f"{GREEN}{'=' * 60}{RESET}")
        sys.exit(0)
    else:
        print(f"{RED}{'=' * 60}{RESET}")
        print(f"{RED}❌ SYSTEM NOT READY - SOME CHECKS FAILED{RESET}")
        print(f"{RED}{'=' * 60}{RESET}")
        sys.exit(1)

if __name__ == "__main__":
    main()
