from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config import Config

# Set up logging configuration
from src.logging_config import setup_logging
logger = setup_logging()

# Create FastAPI app instance with enhanced documentation
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="""
    This API provides access to a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics textbook.

    ## Features
    * Query processing with AI-powered responses
    * Context-aware answers based on indexed content
    * Rate limiting to ensure fair usage
    * Comprehensive error handling

    ## Common Use Cases
    * Answering questions about humanoid robotics
    * Providing explanations of AI concepts
    * Retrieving specific information from the textbook
    """,
    version="1.0.0",
    contact={
        "name": "Development Team",
        "url": "https://github.com/Physical-AI-Humanoid-Robotics",
        "email": "support@example.com",
    },
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    },
    debug=Config.DEBUG
)

# Add CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URLs
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Additional security headers can be added here
)

# Import and include API routes
from src.api.api import router as api_router
app.include_router(api_router, prefix="/api", tags=["api"])

# Basic health check endpoint
@app.get("/health",
         summary="Health check endpoint",
         description="Returns the health status of the API and its dependencies.")
async def health_check():
    """
    Health check endpoint to verify the service is running and dependencies are accessible.

    Returns:
        Dictionary with health status information
    """
    logger.info("Health check endpoint accessed")
    return {"status": "healthy", "service": Config.APP_NAME, "version": "1.0.0"}

# Root endpoint
@app.get("/",
         summary="API Root",
         description="Provides basic information about the API.")
async def root():
    """
    Root endpoint that provides basic information about the API.

    Returns:
        Welcome message with API name
    """
    logger.info("Root endpoint accessed")
    return {"message": f"Welcome to {Config.APP_NAME}", "version": "1.0.0", "docs": "/docs"}

# Startup event to log application startup
@app.on_event("startup")
async def startup_event():
    logger.info("Application startup complete")
    logger.info(f"API Documentation available at: /docs")
    logger.info(f"Redoc documentation available at: /redoc")