// Shared TypeScript interfaces for AI Textbook Platform

export interface User {
  id: string;
  email: string;
  auth_provider: 'email' | 'github';
  hardware_profile?: 'rtx_4090' | 'jetson_orin_nano';
  programming_language?: 'python' | 'cpp';
  created_at: Date;
  last_login?: Date;
}

export interface ChatMessage {
  id: string;
  user_id: string;
  query_text: string;
  response_text: string;
  cited_chapters: string[];
  skills_loaded: string[];
  sanitized_input: string;
  created_at: Date;
}

export interface Citation {
  chapter_id: string;
  section_id: string;
  content_snippet: string;
  chapter_url: string;
}

export interface PersonalizedContent {
  id: string;
  user_id: string;
  chapter_id: string;
  hardware_profile: 'rtx_4090' | 'jetson_orin_nano';
  personalized_mdx: string;
  created_at: Date;
}

export interface TranslatedContent {
  id: string;
  user_id: string;
  chapter_id: string;
  target_language: 'roman_urdu' | 'formal_urdu';
  translated_mdx: string;
  created_at: Date;
}

export interface AuditLog {
  id: string;
  user_id?: string;
  event_type: string;
  event_details: Record<string, any>;
  ip_address?: string;
  created_at: Date;
}
